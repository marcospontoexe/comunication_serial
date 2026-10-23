#!/usr/bin/env python3
"""
Hardware check: NACK instead of ACK when the ESP32 buffer pool is full.

What it checks: since the ACK-on-enqueue fix, serialTask only replies ACK once a
line is in rxQueue; with no free buffer it replies NACK;<id>;NO_BUFFER (or
QUEUE_FULL) INSTEAD of the ACK. The ROS node cannot trigger this (it sends one
message at a time and waits for the reply), so this script talks to the serial
port directly and sends a burst of DISPLAY commands faster than loop() runs them.

Verdict:
  PASSED        every id got an ACK or a NACK (never both, never neither), at least
                one NACK happened, every ACKed id ran (EXEC_OK) and no NACKed id ran;
  FAILED        some id got both ACK and NACK (the order of the old firmware), or
                what ran does not match what was acknowledged;
  INCONCLUSIVE  the pool did not fill (no NACK), a reply was missing, or the ESP
                reported a lost/duplicate line: tune --count / --gap-ms / --first-id.

How to run it, on the robot, parked:
  stop the bridge node first (it must not hold the port)
  python3 tools/check_nack_on_full_pool.py        # ~15 s
  start the bridge node again

Side effects (expected):
  - The script sends NO PING: the emergency stop (CONTROL_STOP) stays engaged for
    the whole run. That is the safe state.
  - If opening the port resets the ESP32, the PINS_OUT outputs pulse for 2 s (the
    firmware boot self-test) and the ESP32 sends its initial PININ events, which
    the script acknowledges.
  - The display ends up showing "T" and the id of the last command that ran.

Exit code: 0 = PASSED, 1 = FAILED, 2 = INCONCLUSIVE or usage error.
Needs only python3 and pyserial (apt package python3-serial, same as the node).
"""

import argparse
import os
import re
import sys
import threading
import time

import serial

# A burst line is ~25 bytes. The ESP32 UART receive buffer holds 256 bytes and
# serialTask drains it every ~50 ms; sending everything at once would overflow that
# buffer and the check would turn into "lost lines" instead of "full pool". With
# 8 ms between lines, ~6 lines (~150 bytes) arrive per serialTask cycle: that fits
# the buffer and is still far faster than loop() runs commands (one per ~50-80 ms).
DEFAULT_GAP_MS = 8


def processes_holding(port):
    """Return (pid, command line) of the other processes that have the port open."""
    target = os.path.realpath(port)   # /dev/ESP is a udev symlink to /dev/ttyUSBx
    found = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        if int(pid) == os.getpid():
            continue
        try:
            fds = os.listdir(f'/proc/{pid}/fd')
        except OSError:
            continue   # another user's process (no permission) or one that just exited
        for fd in fds:
            try:
                if os.readlink(f'/proc/{pid}/fd/{fd}') == target:
                    with open(f'/proc/{pid}/cmdline', 'rb') as f:
                        cmd = f.read().replace(b'\0', b' ').decode(errors='replace')
                    found.append((pid, cmd.strip()))
                    break
            except OSError:
                continue
    return found


class Session:
    """Open port plus a thread that reads the ESP32 lines and ACKs its own MSGs."""

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.write_lock = threading.Lock()   # burst and ACKs come from different threads
        self.lines = []                      # (timestamp, line) of everything the ESP32 sent
        self.stop = threading.Event()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def write(self, text):
        # one whole line at a time, never interleaved with the other thread
        with self.write_lock:
            self.ser.write(text.encode())
            self.ser.flush()

    def _read(self):
        while not self.stop.is_set():
            try:
                raw = self.ser.readline()
            except serial.SerialException:
                break
            if not raw:
                continue
            line = raw.decode(errors='replace').strip()
            self.lines.append((time.monotonic(), line))
            # The ESP32 expects an ACK for its own messages (boot PININ, a button).
            # Without it the ESP32 retransmits and blocks buttonTask; ACKing keeps
            # the run quiet.
            m = re.match(r'^MSG;(\d+);', line)
            if m:
                self.write(f'ACK;{m.group(1)}\n')

    def close(self):
        self.stop.set()
        self.reader.join(timeout=1.0)
        self.ser.close()


def analyse(lines, ids):
    """Classify the ESP32 replies for every id of the burst."""
    by_id = {i: {'ack': 0, 'nack': [], 'ran': 0, 'dup': 0} for i in ids}
    alerts = []   # ESP32 lines that make the check inconclusive
    for _, line in lines:
        m = re.match(r'^ACK;(\d+)$', line)
        if m and int(m.group(1)) in by_id:
            by_id[int(m.group(1))]['ack'] += 1
            continue
        m = re.match(r'^NACK;(\d+);(.*)$', line)
        if m and int(m.group(1)) in by_id:
            by_id[int(m.group(1))]['nack'].append(m.group(2))
            continue
        # the burst payload is "T;<id>", so EXEC_OK tells which id actually ran
        m = re.match(r'^EXEC_OK:DISPLAY:T;(\d+)$', line)
        if m and int(m.group(1)) in by_id:
            by_id[int(m.group(1))]['ran'] += 1
            continue
        m = re.match(r'^DUP:(\d+)$', line)
        if m and int(m.group(1)) in by_id:
            by_id[int(m.group(1))]['dup'] += 1
            continue
        if line.startswith(('ERROR:LINEBUF_OVERFLOW', 'ERROR:MSG_', 'WARN:NO_BUFFER_DROP')):
            alerts.append(line)
    return by_id, alerts


def verdict(by_id, alerts):
    """Return (exit code, list of reasons) from the analysis."""
    failures, inconclusive = [], []
    for i, r in by_id.items():
        if r['ack'] and r['nack']:
            failures.append(f'id {i}: both ACK and NACK (the order of the old firmware)')
        elif r['ack'] > 1:
            failures.append(f'id {i}: {r["ack"]} ACKs for a single send')
        elif not r['ack'] and not r['nack']:
            inconclusive.append(f'id {i}: no reply (lost line?)')
        if r['ack'] and r['ran'] != 1:
            failures.append(f'id {i}: acknowledged but ran {r["ran"]} time(s)')
        if r['nack'] and r['ran']:
            failures.append(f'id {i}: refused with NACK but ran anyway')
        if r['nack'] and any(reason not in ('NO_BUFFER', 'QUEUE_FULL') for reason in r['nack']):
            failures.append(f'id {i}: NACK with an unexpected reason {r["nack"]}')
        if r['dup']:
            inconclusive.append(f'id {i}: DUP (the id was already in the ESP32 window; '
                                'change --first-id)')
    if not any(r['nack'] for r in by_id.values()):
        inconclusive.append('no NACK: the pool did not fill (raise --count or lower --gap-ms)')
    inconclusive += [f'the ESP32 reported: {a}' for a in alerts]
    if failures:
        return 1, failures + inconclusive
    if inconclusive:
        return 2, inconclusive
    return 0, []


def main():
    p = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    p.add_argument('--port', default='/dev/ESP')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--count', type=int, default=20, help='lines in the burst (the pool has 10)')
    p.add_argument('--gap-ms', type=float, default=DEFAULT_GAP_MS,
                   help='pause between burst lines (see DEFAULT_GAP_MS)')
    p.add_argument('--first-id', type=int, default=9000,
                   help='first id; keep it away from the node ids, which restart at 1')
    p.add_argument('--settle-s', type=float, default=6.0,
                   help='wait before the burst (ESP32 boot, if opening the port resets it)')
    p.add_argument('--reply-s', type=float, default=4.0,
                   help='wait after the burst for replies and executions')
    p.add_argument('--show-all', action='store_true', help='print every ESP32 line')
    a = p.parse_args()

    # Two processes reading one port split the bytes between them: the check is void.
    others = processes_holding(a.port)
    if others:
        print(f'{a.port} is open by another process; stop it first '
              '(the bridge node, a serial monitor):')
        for pid, cmd in others:
            print(f'  pid {pid}: {cmd}')
        return 2

    session = Session(a.port, a.baud)
    try:
        print(f'{a.port} open. Waiting {a.settle_s:.0f} s for the ESP32 to settle '
              '(no PING: the emergency stop stays engaged)...')
        time.sleep(a.settle_s)
        reset = any('rst:' in line for _, line in session.lines)
        print('The ESP32 reset when the port opened (boot banner seen).' if reset
              else 'The ESP32 did not reset when the port opened.')

        ids = list(range(a.first_id, a.first_id + a.count))
        burst_start = len(session.lines)
        print(f'Burst: {a.count} DISPLAY commands (ids {ids[0]}..{ids[-1]}), '
              f'one every {a.gap_ms:g} ms...')
        for i in ids:
            session.write(f'MSG;{i};DISPLAY;T;{i}\n')
            time.sleep(a.gap_ms / 1000.0)
        time.sleep(a.reply_s)
    finally:
        session.close()

    burst_lines = session.lines[burst_start:]
    if a.show_all:
        print('\n--- ESP32 lines since the burst ---')
        for _, line in burst_lines:
            print(f'  {line}')

    by_id, alerts = analyse(burst_lines, ids)
    print('\n  id     reply                 ran')
    for i, r in by_id.items():
        reply = ' + '.join((['ACK'] * r['ack']) + [f'NACK({m})' for m in r['nack']]) or '-'
        print(f'  {i:<6} {reply:<21} {"yes" if r["ran"] else "no"}')
    n_ack = sum(1 for r in by_id.values() if r['ack'])
    n_nack = sum(1 for r in by_id.values() if r['nack'])
    print(f'\n  {n_ack} ACKed, {n_nack} NACKed, out of {len(ids)} sent')

    code, reasons = verdict(by_id, alerts)
    print(f'\nVERDICT: {["PASSED", "FAILED", "INCONCLUSIVE"][code]}')
    for reason in reasons:
        print(f'  - {reason}')
    print('\nReminder: start the bridge node again.')
    return code


if __name__ == '__main__':
    sys.exit(main())
