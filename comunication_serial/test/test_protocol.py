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
import queue
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


@pytest.fixture(autouse=True)
def recorded_os_system(monkeypatch):
    """
    Replace os.system with a recorder in EVERY test.

    BUTTON;0 runs 'sudo shutdown now'; without this net, a test that reaches that
    branch would shut down the machine running the suite.
    """
    calls = []
    monkeypatch.setattr(comm.os, 'system', lambda cmd: calls.append(cmd) or 0)
    return calls


class _RecordingLogger:
    """Stand-in for the rclpy logger: keeps (level, message) pairs in a list."""

    def __init__(self, records):
        self._records = records

    def debug(self, msg):
        self._records.append(('debug', msg))

    def info(self, msg):
        self._records.append(('info', msg))

    def warning(self, msg):
        self._records.append(('warning', msg))

    def error(self, msg):
        self._records.append(('error', msg))

    def fatal(self, msg):
        self._records.append(('fatal', msg))


@pytest.fixture
def logged_node(node, monkeypatch):
    """Like ``node``, but with the real generate_log_msg recording into node.logged."""
    monkeypatch.setattr(comm, 'LOG_LEVEL', comm.LogsLevel.debug)   # capture everything
    del node.generate_log_msg   # drop the no-op from the node fixture: the real method runs
    node.logged = []
    node.get_logger = lambda: _RecordingLogger(node.logged)
    node._file_logger = None
    return node


def levels_of(node, fragment):
    """Return the levels of the logged messages that contain the fragment."""
    return [level for level, msg in node.logged if fragment in msg]


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


def test_firmware_nack_reaches_the_waiting_sender(node):
    """The firmware answers NACK instead of ACK when it has no room for the line.

    Unlike the test above, the reply goes through the real ``_reader_thread_fn``:
    the fake port answers every MSG with ``NACK;<id>;NO_BUFFER``, as the firmware
    does when its buffer pool is exhausted.
    """
    incoming = queue.Queue()

    def write(data):
        if data.startswith(b'MSG;'):
            msg_id = data.split(b';')[1]
            incoming.put(b'NACK;' + msg_id + b';NO_BUFFER\n')

    def readline():
        try:
            return incoming.get(timeout=0.05)
        except queue.Empty:
            return b''

    node.ser.write.side_effect = write
    node.ser.readline.side_effect = readline
    reader = threading.Thread(target=node._reader_thread_fn, daemon=True)
    reader.start()
    try:
        started = time.time()
        result = node.send_message_with_ack('PINOUT;0;high', retries=3, timeout=1.0)
        elapsed = time.time() - started
    finally:
        node._stop_event.set()
        reader.join(timeout=1.0)

    assert result is False
    assert node.ser.write.call_count == 1, 'an explicit rejection must not be retried'
    assert elapsed < 0.5, 'the NACK must wake the sender, not the timeout'
    assert node.pending_acks == {}


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


def test_pinin_with_an_invalid_state_is_not_published(node):
    # the state must be exactly high or low; the line is still acknowledged
    feed(node, [b'MSG;10;PININ;IN_2;HIGH\n'])
    node.pinin_pub.publish.assert_not_called()
    acks = [call.args[0] for call in node.ser.write.call_args_list]
    assert acks == [b'ACK;10\n']


def test_power_button_shuts_down_and_is_not_published(node, recorded_os_system):
    # The node reuses one message object for /LEDs and /oled, so record .data at
    # call time; a MagicMock would only keep the object, already overwritten.
    leds, oled = [], []
    node.led_pub.publish.side_effect = lambda m: leds.append(m.data)
    node.oled_pub.publish.side_effect = lambda m: oled.append(m.data)

    feed(node, [b'MSG;11;BUTTON;0\n'])

    assert recorded_os_system == ['sudo shutdown now']
    node.button_pub.publish.assert_not_called()
    assert leds == ['Apagado']
    assert oled == [';Shutting down..']


# ---------------------------------- late NACK ----------------------------------

def test_nack_after_the_ack_from_an_older_firmware_is_logged(logged_node):
    """
    A firmware flashed before the ACK-on-enqueue fix ACKs first and NACKs later.

    The sender already returned True on the ACK; the NACK that follows finds no
    pending entry and must still leave a warning behind.
    """
    node = logged_node
    incoming = queue.Queue()

    def write(data):
        if data.startswith(b'MSG;'):
            msg_id = data.split(b';')[1]
            incoming.put(b'ACK;' + msg_id + b'\n')
            incoming.put(b'NACK;' + msg_id + b';QUEUE_FULL\n')

    def readline():
        try:
            return incoming.get(timeout=0.05)
        except queue.Empty:
            return b''

    node.ser.write.side_effect = write
    node.ser.readline.side_effect = readline
    reader = threading.Thread(target=node._reader_thread_fn, daemon=True)
    reader.start()
    try:
        assert node.send_message_with_ack('LED;Azul', retries=3, timeout=1.0) is True
        deadline = time.time() + 1.0   # the NACK is read after the sender returns
        while not levels_of(node, 'Late NACK') and time.time() < deadline:
            time.sleep(0.01)
    finally:
        node._stop_event.set()
        reader.join(timeout=1.0)

    assert levels_of(node, 'Late NACK') == ['warning']


def test_nack_with_no_sender_waiting_is_logged_as_a_warning(logged_node):
    # e.g. the NACK arrived after the sender's last timeout
    feed(logged_node, [b'NACK;42;NO_BUFFER\n'])
    assert levels_of(logged_node, 'Late NACK id=42') == ['warning']


# --------------------------- ESP diagnostic lines ---------------------------

@pytest.mark.parametrize('line', [
    'EXEC_FAIL:PINOUT_FORMAT:x;high',
    'EXEC_FAIL:PINOUT_RANGE:3;high',
    'EXEC_FAIL:UNKNOWN_CMD:FOO',
    'ERROR:LINEBUF_OVERFLOW',
    'ERROR: lost communication with the OLED display!',
    'WARN:NO_PENDING_SLOT',
])
def test_esp_failure_lines_are_logged_as_warnings(logged_node, line):
    feed(logged_node, [(line + '\n').encode()])
    assert levels_of(logged_node, f'Host received -> {line}') == ['warning']


@pytest.mark.parametrize('line', [
    'EXEC_OK:PINOUT:0;high',
    'GPIO_SETTED_HIGH:32',
    'DUP:7',
])
def test_ordinary_esp_lines_stay_at_debug(logged_node, line):
    feed(logged_node, [(line + '\n').encode()])
    assert levels_of(logged_node, f'Host received -> {line}') == ['debug']


def test_esp_failure_shows_at_the_default_info_level(logged_node, monkeypatch):
    # with the production level (info) the failure shows and the noise stays hidden
    monkeypatch.setattr(comm, 'LOG_LEVEL', comm.LogsLevel.info)
    feed(logged_node, [b'GPIO_SETTED_HIGH:32\n', b'EXEC_FAIL:PINOUT_FORMAT:x;high\n'])
    assert logged_node.logged == [
        ('warning', 'Host received -> EXEC_FAIL:PINOUT_FORMAT:x;high')]
