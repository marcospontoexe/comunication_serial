# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A reliable line-based serial bridge between an ESP32 (FreeRTOS firmware) and ROS 2 (an `rclpy` node). It replaces micro-ROS because the target robot runs `rmw_zenoh_cpp`, which micro-ROS does not support. The repo has two halves that implement **one protocol**:

- [esp32/esp32.ino](esp32/esp32.ino): Arduino-ESP32 firmware, built and flashed with the Arduino IDE (board "ESP32 Dev Module"; libs Adafruit NeoPixel, GFX, SSD1306). There is no CLI build in the repo.
- [comunication_serial/](comunication_serial/): `ament_python` ROS 2 package. All node logic is in a single file, [comunication_serial/comunication_serial/communicator_serial.py](comunication_serial/comunication_serial/communicator_serial.py). The package root is nested one level down.

The package name is spelled `comunication_serial` (single "m") everywhere: package, directory, entry point. Don't "fix" it, because that breaks the ROS package identity.

## Commands

Run these from the package directory `comunication_serial/`, not the repo root:

```bash
pytest test/test_protocol.py -v                       # protocol suite: no ROS, no hardware, only pytest needed
pytest test/test_protocol.py::test_timeout_exhausts_every_attempt   # single test
```

In a ROS 2 workspace (the package copied into `~/ros2_ws/src`):

```bash
colcon build --packages-select comunication_serial
colcon test --packages-select comunication_serial      # also runs ament flake8 + pep257
ros2 launch comunication_serial communicator_serial.launch.py
```

`test_flake8.py` and `test_pep257.py` need ament (ROS installed). ament_flake8 uses max line length 99 and Google import order, and pep257 requires docstrings on public functions and classes. `test_copyright.py` is skipped on purpose.

## Protocol invariants (both sides must stay in sync)

Wire format, one message per `\n`-terminated line: `MSG;<id>;<CMD>;<ARGS...>`, `ACK;<id>`, `NACK;<id>;<REASON>`, `PING`. Any other line is diagnostic text, and the node logs it at debug level.

Each side mirrors the other's reliability layer. A change to one side almost always needs the matching change on the other:

| Concept | Python (`communicator_serial.py`) | Firmware (`esp32.ino`) |
| --- | --- | --- |
| Sender waits for reply | `send_message_with_ack()` + `PendingAck` (`threading.Event` + status) | `send_msg_with_ack_esp()` + `pendingAcks[]` (binary semaphore + status) |
| Dedup window (16 ids) | `_seen_ids` deque, `SEEN_IDS_MAXLEN` | `seenIds[]` ring, `SEEN_IDS_SIZE` |
| Timeouts/retries | `ACK_TIMEOUT`, `DEFAULT_RETRIES`, `HEARTBEAT_INTERVAL` | `timeout_ms`/`retries` defaults, `HEARTBEAT_TIMEOUT_MS` |

Rules that the code and tests depend on:

- **The waiter carries status, not just a wakeup.** A NACK wakes the sender and returns `False`. It is **never retransmitted**, because it signals a lack of resources, not a link loss. Only a timeout causes a retry.
- **Duplicates are re-ACKed but not re-executed or re-published.** The ACK goes out *before* the dedup check, since a lost ACK is the reason the line was resent.
- **The firmware ACKs on reception, not on execution.** `serialTask` ACKs, then enqueues into `rxQueue`; `loop()` executes. It can therefore send `ACK` and then `NACK;<id>;NO_BUFFER|QUEUE_FULL` for the same id, and the Python reader handles a NACK whose id is no longer pending. An id goes into `seenIds` only once it is successfully enqueued.
- **Ids are per-direction counters.** Each side tracks only the ids it generated. The firmware uses `0xFFFFFFFF` as the empty sentinel because 0 is a valid id.
- **Watchdog:** the node sends `PING` every 0.5 s, and the firmware pulls `CONTROL_STOP` low after 1000 ms without one. Expect this when testing by hand from a serial monitor.

## Firmware constraints

- **No dynamic allocation:** no `malloc`, `new` or Arduino `String`. Incoming lines use the fixed `pool[POOL_SIZE][RX_LINE_MAX_LEN]`, with indices passed through `freeQueue` and `rxQueue`.
- **Every serial write goes through `serialSendLine()`/`serialSendLinef()`**, which take `serialMutex`. Tasks run on both cores, so a raw `Serial.print` can interleave mid-line. Multi-line blocks must hold the mutex for the whole block, as in the stack report in `loop()`. Use a mutex, not a binary semaphore, because priority inheritance matters (`serialTask` runs at prio 3). Never write serial from an ISR: set a flag and let a task emit the line.
- **Stack sizes are in bytes** (ESP-IDF port). Any task that can reach `serialSendLinef()` (vsnprintf) needs about 4096 bytes. `loop()` prints every task's high-water mark every 5 s.
- `PINOUT;<idx>` and `BUTTON;<n>` use **indices into `PINS_OUT[]` / `BUTTON_PINS[]`, not GPIO numbers**. `PININ` sends the `#define` name, via `STRINGIFY`. Array counts are derived with `sizeof`.
- LED colour names (`Branco`, `Verde`, ...), `Apagado` and the topic `/desliga_hardware` are Portuguese **protocol values**. Keep them as they are, even though code and comments are in English.

## ROS node structure

Three concurrent contexts share `pending_acks` (guarded by `pending_lock`):

- The `rclpy.spin()` executor runs the subscriber callbacks, which block inside `send_message_with_ack()`.
- `_reader_thread_fn` dispatches ACK/NACK and publishes ESP events. It is the only thread that touches `_seen_ids`, so that needs no lock.
- `_heartbeat_thread_fn` sends the PINGs.

On a timeout, `send_message_with_ack()` registers a **fresh** `PendingAck` for the next attempt, so a late reply cannot end the next wait early.

`BUTTON;0` is the power button. It is **not** published on `/button`. Instead it publishes `Apagado`/shutdown text and runs `sudo shutdown now` (this needs the sudoers rule in the README). Configuration lives in module constants at the top of the file (`PORT='/dev/ESP'`, `BAUD`, `LOG_LEVEL`, `LOG_FILE`), not ROS params.

**Adding a command:** add an `else if` branch in `handle_MSG_command()` (firmware), then a method on `SerialCommNode` that calls `send_message_with_ack()`, plus a subscription. For an ESP→ROS event, add a branch in `_reader_thread_fn` after the dedup check.

## Tests

[test/test_protocol.py](comunication_serial/test/test_protocol.py) loads the module by file path. It stubs `rclpy`, `std_msgs` and `serial` **only when they are missing**. The `node` fixture builds `SerialCommNode` through `__new__`, which skips `__init__` (no port, no threads), and injects attributes by hand. If a tested method starts using a new attribute, add that attribute to the fixture. `feed()` drives the real reader loop by setting `_stop_event` once its input lines run out.

## Docs

[README.md](README.md) restates many code facts in tables: timing constants, the firmware task table (core/prio/stack), topics and QoS, and the list of tests. Update the README when you change those facts.
