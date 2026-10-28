#!/usr/bin/env python3
"""
Hardware check: how the ESP32 waits for the host's ACK (retry, timeout and NACK).

What it checks: a sender on the ESP32 (send_msg_with_ack_esp) sleeps on its own
task notification until ACK;<id> or NACK;<id> arrives, or until the 1 s timeout;
with no reply it resends the same id (3 attempts in all) and gives up. None of this
is visible with the bridge node running: the node is the one replying, and the
ESP32's "...: ACK received from the host" lines only reach its log at debug level.
This script takes the node's place: it resets the ESP32 through DTR (as the node
does) and answers the first three ESP32 events, each in a different way:

  event 1  no reply        3 sends of the same id, ~1 s apart, then
                           "...: no ACK from the host (timeout)". The script then
                           sends that id's ACK late, and the ESP32 must ignore it.
  event 2  ACK, twice      1 send, then "...: ACK received from the host".
  event 3  NACK            1 send only (a NACK is never retransmitted), then
                           "...: no ACK from the host (timeout)".

Event 1 is the boot PININ. This firmware monitors a single pin (MONITOR_PINS has
only IN_2), so the script asks for two button presses for events 2 and 3; with
more monitored pins, the boot PININ events cover all three.

Verdict:
  PASSED        the three events behaved as above;
  FAILED        one of them did not (number of sends, wait or result);
  INCONCLUSIVE  an event did not arrive in time, the power button was pressed, or
                the ESP32 reported a lost line.

What it cannot prove: that a late notification is discarded through the id packed
into its value. That race lasts microseconds and cannot be triggered from outside.
The double ACK of event 2 only exercises it by luck; if it does and the filter is
broken, event 3 ends with "ACK received" instead of the NACK, and the verdict is
FAILED. The previous firmware (binary semaphores) passes as well: this checks that
the mechanism works, not the difference between the two.

How to run it, on the robot, parked:
  stop the bridge node first (it must not hold the port)
  python3 tools/check_ack_wait.py            # ~10 s, plus the button presses
  start the bridge node again

Side effects (expected):
  - The ESP32 resets: the PINS_OUT outputs pulse for 2 s (the firmware boot
    self-test), as when the node starts.
  - The script sends NO PING: the emergency stop (CONTROL_STOP) stays engaged for
    the whole run. That is the safe state.
  - Never press button 0 (BUTTON_0): it is the power button, and the ESP32 cuts
    the power after its countdown whatever the reply.

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

# Attempts of the events used here (boot PININ, PININ and buttons 1-4): buttonTask
# calls send_msg_with_ack_esp(payload, 3, 1000).
ATTEMPTS = 3

# Accepted range for one unanswered attempt. The floor catches a wait that returns
# before the timeout (for example a notification that was not the reply, taken as a
# timeout); the ceiling, a wait stuck past it.
WAIT_MIN_S = 0.9
WAIT_MAX_S = 1.4

# A reply (ACK or NACK) must end the wait well before the timeout. serialTask polls
# the port every ~50 ms, so a reply takes at most ~100 ms to wake the sender.
REPLY_MAX_S = 0.5

# How the script answers the events under test, in arrival order.
PLAN = ('none', 'double_ack', 'nack')
DESCRIPTION = {'none': 'none (late ACK afterwards)', 'double_ack': 'ACK, twice',
               'nack': 'NACK'}

# buttonTask sends one event at a time and then prints one of these result lines.
RESULT_RE = re.compile(r'^(INITIAL_PININ_EVENT|BUTTON_EVENT|PININ): (.*)$')
ACK_TEXT = 'ACK received from the host'

# The ESP32 is ready ~3 s after the DTR reset and sends its boot PININ events 0.5 s
# apart. The script asks for a button press once nothing is in flight and the
# ESP32 has been quiet for QUIET_S, but not before BOOT_S after the reset.
BOOT_S = 5.0
QUIET_S = 1.5


def processes_holding(port):
    """Return (pid, command line) of the other processes that have the port open."""
    # Same function as in check_nack_on_full_pool.py: each tool is copied to the
    # robot on its own.
    target = os.path.realpath(port)   # /dev/ESP is a udev symlink to /dev/ttyUSBx
    found = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        if int(pid) == os.getpid():
            continue
        try:
            fds = os.listdir(f'/proc/{pid}/fd')
        except OSError:
            continue   # another user's process (no permission) or one that just ended
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
    """Open port plus a thread that reads the ESP32 lines and answers per PLAN."""

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.write_lock = threading.Lock()   # replies leave from the reader thread
        self.lines = []                      # (time, line) of everything the ESP32 sent
        self.tracked = []                    # ids of the events under test, in order
        self.payloads = {}                   # id -> payload (e.g. "BUTTON;1")
        self.results = 0                     # result lines seen so far
        self.other_msgs = []                 # MSG lines not under test (acknowledged)
        self.dtr_ok = self._reset_esp()
        self.opened = time.monotonic()
        self.last_activity = self.opened     # last send or result of an event
        self.stop = threading.Event()
        self.done = threading.Event()        # the three events have their result
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _reset_esp(self):
        # Same as the node (communicator_serial.py): drop whatever was buffered and
        # reset the ESP32 through DTR, so that it sends its boot PININ events.
        self.ser.reset_input_buffer()
        try:
            self.ser.setDTR(False)
            time.sleep(0.1)
            self.ser.setDTR(True)
            return True
        except (OSError, serial.SerialException):
            return False   # a port without DTR: the ESP32 is not reset from here

    def write(self, text):
        # one write at a time, so lines from different threads never interleave
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
            m = re.match(r'^MSG;(\d+);(.*)$', line)
            if m:
                self._answer(m.group(1), m.group(2))
            elif RESULT_RE.match(line):
                self._result()

    def _answer(self, msg_id, payload):
        # The first send of an event fixes its place in PLAN. Button 0 is the power
        # button and is never under test.
        under_test = payload.startswith('PININ;') or (
            payload.startswith('BUTTON;') and payload != 'BUTTON;0')
        if msg_id not in self.tracked and under_test and len(self.tracked) < len(PLAN):
            self.tracked.append(msg_id)
            self.payloads[msg_id] = payload
        if msg_id in self.tracked:
            self.last_activity = time.monotonic()
            plan = PLAN[self.tracked.index(msg_id)]
            if plan == 'double_ack':
                # Two ACKs in one write: the second may reach serialTask before
                # buttonTask frees its slot and leave a late notification behind,
                # which event 3 must then discard.
                self.write(f'ACK;{msg_id}\nACK;{msg_id}\n')
            elif plan == 'nack':
                self.write(f'NACK;{msg_id};TEST\n')
            # 'none': stay silent; the ESP32 must resend and give up
        else:
            # Any other event: acknowledge it, so the ESP32 does not keep resending.
            self.other_msgs.append(payload)
            self.write(f'ACK;{msg_id}\n')

    def _result(self):
        # buttonTask sends its events one at a time, so the n-th result line is the
        # result of the n-th event.
        n = self.results
        self.results += 1
        self.last_activity = time.monotonic()
        if n == 0 and self.tracked:
            # The ESP32 gave up on event 1: its late ACK arrives now. Nobody waits
            # for that id any more, so the ESP32 must ignore it; if it woke the next
            # sender, event 2 would end without the script's reply.
            self.write(f'ACK;{self.tracked[0]}\n')
        if self.results >= len(PLAN):
            self.done.set()

    def waiting_for_press(self):
        """Tell whether the next event has to come from a button press."""
        # Nothing in flight, the plan still needs events, and the ESP32 has been
        # quiet for a while (and is past its boot): no boot PININ is left to come.
        quiet_since = max(self.last_activity, self.opened + BOOT_S)
        return (len(self.tracked) == self.results < len(PLAN)
                and time.monotonic() - quiet_since >= QUIET_S)

    def close(self):
        self.stop.set()
        self.reader.join(timeout=1.0)
        self.ser.close()


def analyse(lines, tracked):
    """Collect, for each event under test, the send times and its result line."""
    sends = {i: [] for i in tracked}
    results = []   # (time, line) of the result lines, in order
    alerts = []    # ESP32 lines that make the check inconclusive
    for t, line in lines:
        m = re.match(r'^MSG;(\d+);', line)
        if m and m.group(1) in sends:
            sends[m.group(1)].append(t)
        elif RESULT_RE.match(line):
            results.append((t, line))
        elif line.startswith(('ERROR:LINEBUF_OVERFLOW', 'ERROR:MSG_', 'WARN:NO_')):
            alerts.append(line)
    return sends, results, alerts


def judge(plan, sends, result):
    """Return the problems of one event (an empty list means as expected)."""
    t_res, line = result
    acked = line.endswith(ACK_TEXT)
    problems = []
    if plan == 'none':
        if len(sends) != ATTEMPTS:
            problems.append(f'{len(sends)} send(s), expected {ATTEMPTS}')
        # each wait runs from one send to the next; the last one, to the result
        waits = [b - a for a, b in zip(sends, sends[1:] + [t_res])]
        off = [f'{w:.2f}' for w in waits if not WAIT_MIN_S <= w <= WAIT_MAX_S]
        if off:
            problems.append(f'wait outside {WAIT_MIN_S}-{WAIT_MAX_S} s: {", ".join(off)} s')
        if acked:
            problems.append('the ESP32 says "ACK received", but the script never replied')
    else:
        if len(sends) != 1:
            problems.append(f'{len(sends)} sends, expected 1 (the reply did not end the wait)')
        if t_res - sends[0] > REPLY_MAX_S:
            problems.append(f'{t_res - sends[0]:.2f} s from send to result '
                            f'(expected < {REPLY_MAX_S} s)')
        if plan == 'double_ack' and not acked:
            problems.append('the script sent ACK, but the ESP32 says "no ACK"')
        if plan == 'nack' and acked:
            problems.append('the script sent NACK, but the ESP32 says "ACK received" '
                            '(a late notification from event 2 taken as the reply?)')
    return problems


def main():
    p = argparse.ArgumentParser(description=__doc__.strip().split('\n')[0])
    p.add_argument('--port', default='/dev/ESP')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--timeout-s', type=float, default=60.0,
                   help='maximum wait for the three events, button presses included')
    p.add_argument('--show-all', action='store_true',
                   help='print every ESP32 line, with its time')
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
        print(f'{a.port} open; '
              f'{"ESP32 reset through DTR" if session.dtr_ok else "the port refused DTR"}. '
              f'Waiting for {len(PLAN)} events (up to {a.timeout_s:.0f} s; no PING: '
              'the emergency stop stays engaged)...')
        deadline = time.monotonic() + a.timeout_s
        prompted_for = None   # how many events had arrived at the last prompt
        while not session.done.wait(0.1) and time.monotonic() < deadline:
            n = len(session.tracked)
            if session.waiting_for_press() and prompted_for != n:
                print(f'  Press button 1, 2, 3 or 4 once (NOT button 0, the power '
                      f'button): event {n + 1} of {len(PLAN)}.')
                prompted_for = n
        time.sleep(0.5)   # catch what comes right after the last result
    finally:
        session.close()

    lines = session.lines
    if a.show_all:
        t0 = lines[0][0] if lines else 0.0
        print('\n--- ESP32 lines (s since the first one) ---')
        for t, line in lines:
            print(f'  {t - t0:7.3f}  {line}')

    sends, results, alerts = analyse(lines, session.tracked)
    failures, inconclusive = [], []
    print()
    for k, plan in enumerate(PLAN):
        if k >= len(session.tracked) or k >= len(results):
            inconclusive.append(f'event {k + 1}: did not arrive (or had no result) in time')
            continue
        msg_id = session.tracked[k]
        tx = sends[msg_id]   # never empty: an id is tracked on a send that was seen
        t_res, line = results[k]
        # times relative to the event's first send, to show the waits
        marks = ' '.join(f'{t - tx[0]:.2f}' for t in tx)
        print(f'Event {k + 1} (id {msg_id}, {session.payloads[msg_id]})')
        print(f'  script reply:  {DESCRIPTION[plan]}')
        print(f'  ESP32 sends:   {len(tx)}, at t = {marks} s')
        print(f'  result:        "{RESULT_RE.match(line).group(2)}" at t = {t_res - tx[0]:.2f} s')
        problems = judge(plan, tx, results[k])
        print(f'  {"FAILED: " + "; ".join(problems) if problems else "ok"}')
        failures += [f'event {k + 1}: {x}' for x in problems]

    if 'BUTTON;0' in session.other_msgs:
        inconclusive.append('the power button (BUTTON;0) was pressed during the check')
    inconclusive += [f'the ESP32 reported: {x}' for x in alerts]
    if session.other_msgs:
        print(f'\nOther ESP32 events, acknowledged: {", ".join(session.other_msgs)}')

    if failures:
        code, reasons = 1, failures + inconclusive
    elif inconclusive:
        code, reasons = 2, inconclusive
    else:
        code, reasons = 0, []
    print(f'\nVERDICT: {["PASSED", "FAILED", "INCONCLUSIVE"][code]}')
    for reason in reasons:
        print(f'  - {reason}')
    print('\nReminder: start the bridge node again.')
    return code


if __name__ == '__main__':
    sys.exit(main())
