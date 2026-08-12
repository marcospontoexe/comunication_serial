"""Protocol-level tests: ACK/NACK handling and id deduplication.

These exercise the real ``send_message_with_ack`` and ``_reader_thread_fn`` from
``communicator_serial.py`` with no hardware attached. Any of ``rclpy``,
``std_msgs`` or ``serial`` that is missing gets a minimal stub, so the suite also
runs in a bare CI container without a ROS installation.

The node is built with ``__new__`` to skip ``__init__``, which would open the
serial port and start the worker threads; only the attributes the tested methods
touch are injected.

Run with:  pytest test/test_protocol.py
"""

import importlib.util
from pathlib import Path
import sys
import threading
import time
import types
from unittest.mock import MagicMock

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_ROOT / 'comunication_serial' / 'communicator_serial.py'


def _stub_missing_modules():
    """Install minimal stubs, but only for modules that cannot be imported here."""

    def missing(name):
        try:
            __import__(name)
            return False
        except ImportError:
            return True

    if missing('serial'):
        mod = types.ModuleType('serial')
        mod.Serial = MagicMock()
        sys.modules['serial'] = mod

    if missing('rclpy'):
        for name in ('rclpy', 'rclpy.node', 'rclpy.logging', 'rclpy.qos'):
            sys.modules[name] = types.ModuleType(name)
        sys.modules['rclpy.node'].Node = type('Node', (), {})

        class _Severity:
            DEBUG = INFO = WARN = ERROR = FATAL = 0

        sys.modules['rclpy.logging'].LoggingSeverity = _Severity
        for attr in ('QoSProfile', 'QoSReliabilityPolicy',
                     'QoSHistoryPolicy', 'QoSDurabilityPolicy'):
            setattr(sys.modules['rclpy.qos'], attr, MagicMock())

    if missing('std_msgs.msg'):
        sys.modules.setdefault('std_msgs', types.ModuleType('std_msgs'))
        sys.modules['std_msgs.msg'] = types.ModuleType('std_msgs.msg')
        for attr in ('String', 'Int32', 'Bool'):
            setattr(sys.modules['std_msgs.msg'], attr, MagicMock())


def _load_module():
    _stub_missing_modules()
    spec = importlib.util.spec_from_file_location('communicator_serial', MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


comm = _load_module()


@pytest.fixture
def node():
    """Build a SerialCommNode carrying only the state the tested methods need."""
    n = comm.SerialCommNode.__new__(comm.SerialCommNode)
    n.pending_acks = {}
    n.pending_lock = threading.Lock()
    n._stop_event = threading.Event()
    n._seen_ids = comm.deque(maxlen=comm.SEEN_IDS_MAXLEN)
    n.generate_log_msg = lambda *args, **kwargs: None
    n.ser = MagicMock()
    n.button_pub = MagicMock()
    n.pinin_pub = MagicMock()
    n.led_pub = MagicMock()
    n.oled_pub = MagicMock()
    return n


def reply(node, status, reason='', delay=0.05):
    """Simulate the reader thread answering the pending message."""

    def worker():
        time.sleep(delay)
        with node.pending_lock:
            ids = list(node.pending_acks.keys())
            pending = node.pending_acks.pop(ids[0], None) if ids else None
        if pending:
            pending.status = status
            pending.reason = reason
            pending.event.set()

    threading.Thread(target=worker, daemon=True).start()


def feed(node, lines):
    """Drive the real _reader_thread_fn with the given serial lines."""
    remaining = list(lines)

    def readline():
        if remaining:
            return remaining.pop(0)
        node._stop_event.set()   # input exhausted: end the loop
        return b''

    node.ser.readline.side_effect = readline
    node._reader_thread_fn()
    return node


# --------------------------- send_message_with_ack ---------------------------

def test_ack_returns_true_after_a_single_transmission(node):
    reply(node, comm.PendingAck.ACK)
    assert node.send_message_with_ack('LED;Verde', retries=3, timeout=1.0) is True
    assert node.ser.write.call_count == 1


def test_nack_returns_false_and_is_not_retransmitted(node):
    reply(node, comm.PendingAck.NACK, reason='NO_BUFFER')
    started = time.time()
    result = node.send_message_with_ack('PINOUT;0;high', retries=3, timeout=1.0)
    elapsed = time.time() - started

    assert result is False
    assert node.ser.write.call_count == 1, 'an explicit rejection must not be retried'
    assert elapsed < 0.5, 'a NACK must return immediately, not wait for the timeout'


def test_timeout_exhausts_every_attempt(node):
    assert node.send_message_with_ack('DISPLAY;a;b', retries=3, timeout=0.1) is False
    assert node.ser.write.call_count == 3


def test_ack_on_the_last_attempt_still_succeeds(node):
    reply(node, comm.PendingAck.ACK, delay=0.25)   # only answers after two timeouts
    assert node.send_message_with_ack('LED;Azul', retries=3, timeout=0.1) is True
    assert node.ser.write.call_count == 3


def test_pending_table_is_left_clean(node):
    reply(node, comm.PendingAck.ACK)
    node.send_message_with_ack('LED;Verde', retries=3, timeout=1.0)
    assert node.pending_acks == {}


# ------------------------------- deduplication -------------------------------

def test_duplicate_button_is_published_once_and_acked_twice(node):
    feed(node, [b'MSG;5;BUTTON;2\n', b'MSG;5;BUTTON;2\n'])

    assert node.button_pub.publish.call_count == 1, 'a retransmission must not be published again'

    acks = [call.args[0] for call in node.ser.write.call_args_list]
    assert acks == [b'ACK;5\n', b'ACK;5\n'], 'a duplicate must still be re-acknowledged'


def test_distinct_ids_are_both_published(node):
    feed(node, [b'MSG;7;BUTTON;1\n', b'MSG;8;BUTTON;3\n'])
    assert node.button_pub.publish.call_count == 2


def test_duplicate_pinin_is_published_once(node):
    feed(node, [b'MSG;9;PININ;IN_2;high\n', b'MSG;9;PININ;IN_2;high\n'])
    assert node.pinin_pub.publish.call_count == 1


def test_id_reused_after_the_window_expires_is_treated_as_new(node):
    window = comm.SEEN_IDS_MAXLEN
    lines = [b'MSG;100;BUTTON;1\n']
    lines += [f'MSG;{i};BUTTON;1\n'.encode() for i in range(200, 200 + window)]
    lines += [b'MSG;100;BUTTON;1\n']   # id 100 has fallen out of the window by now

    feed(node, lines)

    assert node.button_pub.publish.call_count == 1 + window + 1
    assert len(node._seen_ids) == window, 'the window must never grow past its limit'
