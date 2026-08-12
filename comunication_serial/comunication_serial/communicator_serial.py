"""
communicator_serial.py

Reliable communication with an ESP32 over USB serial.

- Outgoing message format: "MSG;<id>;COMMAND;ARGS\n"
    - Hardware shutdown:      "MSG;<id>;POWER_OFF"
    - Heartbeat:              "PING\n" sent every 0.5 s (no ACK)
    - LED strip colour:       "MSG;<id>;LED;<color>"
    - OLED display text:      "MSG;<id>;DISPLAY;<line_1>;<line_2>"
    - Output pin state:       "MSG;<id>;PINOUT;<pin>;<state>"
          (<pin> is a zero-based index, NOT the microcontroller GPIO number;
           <state> is high or low)

- Incoming message format:
    - ESP acknowledged a message from ROS:  "ACK;<id>\n"
    - Buffer/queue unavailable:             "NACK;<id>;REASON"
    - Button state:                         "MSG;<id>;BUTTON;<button>"
    - Input pin state change:               "MSG;<id>;PININ;<pin_name>;<high/low>"
          (<pin_name> is the name of the #define constant in the firmware)

- Automatic retransmission (retries) while waiting for ACK;<id>
- Detailed logging through the rclpy logger, with an optional rotating file
"""

import serial
import threading
import time
import logging
import os
import rclpy
from collections import deque
from logging.handlers import RotatingFileHandler
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import String as RosString
from std_msgs.msg import Int32 as RosInt32
from std_msgs.msg import Bool as RosBool
from enum import Enum


class LogsLevel(Enum):
    debug = 0
    info = 1
    warning = 2
    error = 3
    fatal = 4


# Maps the internal level onto the rclpy severity. Used to configure the node
# logger so that LOG_LEVEL alone decides what gets emitted.
ROS_SEVERITY = {
    LogsLevel.debug:   LoggingSeverity.DEBUG,
    LogsLevel.info:    LoggingSeverity.INFO,
    LogsLevel.warning: LoggingSeverity.WARN,
    LogsLevel.error:   LoggingSeverity.ERROR,
    LogsLevel.fatal:   LoggingSeverity.FATAL,
}

# Maps the internal level onto the stdlib logging levels (only used for the
# optional log file).
STD_LEVEL = {
    LogsLevel.debug:   logging.DEBUG,
    LogsLevel.info:    logging.INFO,
    LogsLevel.warning: logging.WARNING,
    LogsLevel.error:   logging.ERROR,
    LogsLevel.fatal:   logging.CRITICAL,
}

# Settings
PORT = "/dev/ESP"   # adjust to your system; see the udev section in the README
BAUD = 115200
HEARTBEAT_INTERVAL = 0.5      # seconds between PINGs
ACK_TIMEOUT = 1.0
DEFAULT_RETRIES = 3
idCont = 0
LOG_LEVEL = LogsLevel.info

# How many received ids to remember in order to recognise retransmissions. A
# retransmission happens a few seconds after the original, so the window only has
# to cover the traffic within that interval.
SEEN_IDS_MAXLEN = 16

# Optional rotating log file. None = only the rclpy logger, which already writes
# to the console and to /rosout. Set a path to keep history on disk, for example
# "/var/log/serial_comm_node.log".
LOG_FILE = None
LOG_FILE_MAX_BYTES = 5 * 1024 * 1024
LOG_FILE_BACKUP_COUNT = 3


class PendingAck:
    """State of a sent message that is waiting for a reply.

    Mirrors the PendingAck struct in the firmware: the Event wakes whoever is
    waiting, and the status says *why* it woke up. Without the status a NACK (an
    explicit rejection) would be indistinguishable from an ACK, and the sender
    would report success for a message the other side threw away.
    """
    __slots__ = ('event', 'status', 'reason')

    PENDING = 0
    ACK = 1
    NACK = 2

    def __init__(self):
        self.event = threading.Event()
        self.status = PendingAck.PENDING
        self.reason = ""


class SerialCommNode(Node):
    """
    ROS 2 node wrapping the reliable serial link to the ESP32.

    - Two worker threads (reader_thread and heartbeat_thread)
    - Subscribes to:
        - /LEDs (std_msgs/String) -> takes a colour name and sends the LED command
        - /desliga_hardware (std_msgs/Bool) -> on True, asks the ESP to start a safe shutdown
        - /oled (std_msgs/String) -> takes a message and shows it on the OLED display
        - /pin_out (std_msgs/String) -> changes an output pin (e.g. "0;high")

    - Publishes to:
        - /button (std_msgs/Int32) -> which button was pressed, on a BUTTON event from the ESP
        - /LEDs (std_msgs/String) -> publishes "Apagado" when the robot shuts down safely
        - /oled (std_msgs/String) -> clears the OLED display on shutdown
        - /pin_in (std_msgs/String) -> input pin state (e.g. "IN_2;high")
    """

    def __init__(self):
        super().__init__('serial_comm_node')

        # Logging: the rclpy logger is the default sink (console + /rosout).
        # The node logger severity is derived from LOG_LEVEL so that this single
        # constant decides what is emitted — without it, rclpy would drop debug
        # messages on its own.
        self.get_logger().set_level(ROS_SEVERITY[LOG_LEVEL])

        # Optional rotating file, through the stdlib logging module. Disabled by
        # default (LOG_FILE = None); it must exist before the first log call.
        self._file_logger = None
        if LOG_FILE:
            self._file_logger = logging.getLogger('serial_comm_node')
            self._file_logger.setLevel(STD_LEVEL[LOG_LEVEL])
            self._file_logger.propagate = False   # do not duplicate into the root logger
            if not self._file_logger.handlers:
                handler = RotatingFileHandler(
                    LOG_FILE,
                    maxBytes=LOG_FILE_MAX_BYTES,
                    backupCount=LOG_FILE_BACKUP_COUNT,
                )
                handler.setFormatter(logging.Formatter('%(asctime)s [%(levelname)s] %(message)s'))
                self._file_logger.addHandler(handler)

        # Serial port and control structures
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=1)
        except Exception as e:
            # if the port cannot be opened, log it and re-raise for visibility
            self.generate_log_msg(f"Failed to open serial port {PORT}: {e}", LogsLevel.error)
            raise

        # toggle DTR to reset the microcontroller, so it does not stay in bootloader mode
        try:
            self.ser.setDTR(False)
            time.sleep(0.1)
            self.ser.setDTR(True)
        except Exception:
            # not critical; just log it
            self.generate_log_msg("Failed to toggle DTR", LogsLevel.error)

        # msg_id (str) -> PendingAck. Tracks the messages we sent that are still
        # awaiting a reply. The reader thread fills in the status (ACK or NACK) and
        # signals the Event; the sender wakes up and reads the status to find out
        # whether the message was accepted or refused.
        self.pending_acks = {}
        # pending_lock guards concurrent access to the dictionary between the reader
        # thread and whichever thread is sending.
        self.pending_lock = threading.Lock()

        # Window of ids already received from the ESP, used to recognise
        # retransmissions. The ESP resends the same line (same id) when our ACK does
        # not arrive in time; without this window the event would be published twice.
        # Only touched by _reader_thread, so it needs no lock.
        self._seen_ids = deque(maxlen=SEEN_IDS_MAXLEN)

        # Stop flag shared by the worker threads
        self._stop_event = threading.Event()

        # QoS profiles
        transient_local_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        volatile_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # ROS publishers
        self.button_pub = self.create_publisher(RosInt32, 'button', transient_local_qos)
        self.led_pub = self.create_publisher(RosString, 'LEDs', transient_local_qos)
        self.oled_pub = self.create_publisher(RosString, 'oled', transient_local_qos)
        self.pinin_pub = self.create_publisher(RosString, 'pin_in', transient_local_qos)

        # ROS subscribers
        self.create_subscription(RosString, 'LEDs', self._led_callback, transient_local_qos)
        self.create_subscription(RosBool, 'desliga_hardware', self._general_off_sub_callback, volatile_qos)
        self.create_subscription(RosString, 'oled', self._display_sub_callback, transient_local_qos)
        self.create_subscription(RosString, 'pin_out', self._pin_out_sub_callback, volatile_qos)

        # start the worker threads
        self._reader_thread = threading.Thread(target=self._reader_thread_fn, daemon=True)
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_thread_fn, daemon=True)
        self._reader_thread.start()
        self._heartbeat_thread.start()

        self.generate_log_msg(f"serial_comm_node started. port={PORT} baud={BAUD}", LogsLevel.info)

    def _reader_thread_fn(self):
        """Reads lines from the serial port and dispatches ACK/NACK/events/logs."""
        while not self._stop_event.is_set():
            try:
                line = self.ser.readline()
            except Exception as e:
                self.generate_log_msg(f"Error reading the serial port: {e}", LogsLevel.error)
                continue
            if not line:    # readline() returned b'', usually a timeout: nothing to process
                continue

            try:
                # errors="ignore" drops invalid bytes; strip() removes surrounding
                # whitespace and the trailing \r\n
                s = line.decode(errors="ignore").strip()
            except Exception:
                self.generate_log_msg(f"Could not decode line: {line}", LogsLevel.error)
                continue

            # The ESP sent a MSG to the host: acknowledge it and process it
            if s.startswith("MSG;"):
                # Format: MSG;<id>;REST...
                # split with maxsplit=2 keeps the command and its arguments together
                parts = s.split(";", 2)
                # parts[0] is "MSG", parts[1] is <id>, parts[2] is the command plus arguments

                if len(parts) >= 2:
                    msg_id = parts[1]
                    self.generate_log_msg(f"MSG from the ESP (id={msg_id}) -> {s}", LogsLevel.debug)
                    try:
                        # acknowledge reception straight away
                        self.ser.write(f"ACK;{msg_id}\n".encode())
                        self.ser.flush()
                        self.generate_log_msg(f"ACK sent to the ESP, id: {msg_id}", LogsLevel.debug)
                    except Exception as e:
                        self.generate_log_msg(f"Error sending the ACK to the ESP: {e}", LogsLevel.error)

                    # Deduplication: if this id was already processed, the line is a
                    # retransmission caused by an ACK of ours that got lost. The ACK
                    # above has already been resent (that is what the ESP is waiting
                    # for); publishing again would deliver the same event twice to ROS
                    # — a single button press would become two BUTTON messages.
                    if msg_id in self._seen_ids:
                        self.generate_log_msg(
                            f"DUP: MSG id={msg_id} already processed (ESP retransmission); re-acknowledged and ignored",
                            LogsLevel.warning)
                        continue
                    self._seen_ids.append(msg_id)

                    # Beyond logging, the events are translated into ROS topics:
                    #   BUTTON -> published on /button (Int32).  Example: MSG;<id>;BUTTON;<n>
                    #   PININ  -> published on /pin_in (String). Example: MSG;<id>;PININ;IN_2;high
                    part_rest = parts[2] if len(parts) >= 3 else ""
                    # split the command from its arguments
                    rest_parts = part_rest.split(";", 1)
                    if len(rest_parts) >= 1:
                        command = rest_parts[0]
                        args = rest_parts[1] if len(rest_parts) == 2 else ""
                        if command == "PININ":
                            # args: "<pin_name>;<high|low>", e.g. "IN_2;high"
                            arg_parts = args.split(";", 1)
                            if len(arg_parts) == 2 and arg_parts[1] in ("high", "low"):
                                try:
                                    msg_pinin = RosString()
                                    msg_pinin.data = f"{arg_parts[0]};{arg_parts[1]}"
                                    self.pinin_pub.publish(msg_pinin)
                                    self.generate_log_msg(
                                        f"Published on /pin_in: {msg_pinin.data}", LogsLevel.info)
                                except Exception as e:
                                    self.generate_log_msg(f"Error publishing on /pin_in: {e}", LogsLevel.error)
                            else:
                                self.generate_log_msg(f"Malformed PININ: {args}", LogsLevel.warning)

                        if command == "BUTTON":
                            # args is the button index, e.g. "0" or "1"
                            arg_parts = args.split(";", 1)
                            if len(arg_parts) >= 1 and arg_parts[0].isdigit():
                                button = int(arg_parts[0])
                                try:
                                    msg = RosInt32()
                                    if button == 0:   # button that shuts the robot down
                                        msgstr = RosString()
                                        msgstr.data = "Apagado"
                                        self.led_pub.publish(msgstr)
                                        msgstr.data = ";Shutting down.."
                                        self.oled_pub.publish(msgstr)
                                        self.generate_log_msg("Shutting down with sudo shutdown now...", LogsLevel.info)
                                        os.system("sudo shutdown now")
                                    else:
                                        msg.data = button      # publish the index of the pressed button
                                        self.button_pub.publish(msg)
                                        self.generate_log_msg(f"Published on /button: {msg.data}", LogsLevel.info)
                                except Exception as e:
                                    self.generate_log_msg(f"Error publishing: {e}", LogsLevel.error)
                            else:
                                self.generate_log_msg(f"Malformed BUTTON: {args}", LogsLevel.warning)

                    else:
                        self.generate_log_msg(f"MSG without a command: {s}", LogsLevel.warning)
                else:
                    self.generate_log_msg(f"Malformed MSG from the ESP: {s}", LogsLevel.warning)
                continue

            # ACK for a message we sent earlier (host -> ESP)
            if s.startswith("ACK;"):
                ack_id = s[4:]
                self.generate_log_msg(f"ACK received: {ack_id}", LogsLevel.debug)
                with self.pending_lock:   # pop under the lock so no other thread reuses the id
                    pending = self.pending_acks.pop(ack_id, None)
                if pending:  # mark acceptance and wake whoever waits in send_message_with_ack
                    pending.status = PendingAck.ACK
                    pending.event.set()
            elif s.startswith("NACK;"):
                # format: NACK;<id>;REASON  (maxsplit=2 keeps a REASON containing ';' intact)
                parts = s.split(";", 2)
                if len(parts) >= 3:
                    nack_id = parts[1]
                    reason = parts[2]
                    self.generate_log_msg(f"NACK received id={nack_id} reason={reason}", LogsLevel.warning)
                    with self.pending_lock:
                        pending = self.pending_acks.pop(nack_id, None)
                    if pending:
                        # Wake the waiter, but flag a REJECTION: the sender must not
                        # treat a NACK as successful delivery.
                        pending.status = PendingAck.NACK
                        pending.reason = reason
                        pending.event.set()
                    else:
                        # The firmware acknowledges on reception and only then tries to
                        # enqueue; when there is no buffer, the NACK arrives with the id
                        # already acknowledged and removed from here. The message was
                        # accepted and then dropped — we log it explicitly so the case
                        # does not go unnoticed.
                        self.generate_log_msg(
                            f"NACK id={nack_id} arrived after the ACK: command dropped by the firmware ({reason})",
                            LogsLevel.warning)
                else:
                    self.generate_log_msg(f"Malformed NACK: {s}", LogsLevel.warning)
            else:
                # any other line is diagnostic output from the ESP
                self.generate_log_msg(f"Host received -> {s}", LogsLevel.debug)

    def _heartbeat_thread_fn(self):
        """Sends PING every HEARTBEAT_INTERVAL seconds."""
        while not self._stop_event.is_set():
            try:
                self.ser.write(b"PING\n")
                self.ser.flush()   # force an immediate send
                self.generate_log_msg("PING sent", LogsLevel.debug)
            except Exception as e:
                self.generate_log_msg(f"Error sending PING: {e}", LogsLevel.error)
            # short sleeps so shutdown stays responsive
            for _ in range(int(HEARTBEAT_INTERVAL * 10)):
                if self._stop_event.is_set():
                    break
                time.sleep(0.1)

    def _generate_msg_id(self):
        """Returns the next message id (incrementing counter)."""
        global idCont
        idCont = idCont + 1
        if idCont >= 4294967295:
            idCont = 0
        return idCont

    def generate_log_msg(self, msg, log_type):
        """Emits a log message honouring the LOG_LEVEL hierarchy.

        Default sink: the rclpy logger (console and /rosout). If LOG_FILE is set,
        the same message also goes to the rotating file.
        """
        if log_type.value < LOG_LEVEL.value:
            return

        ros_logger = self.get_logger()
        if log_type == LogsLevel.debug:
            ros_logger.debug(msg)
        elif log_type == LogsLevel.info:
            ros_logger.info(msg)
        elif log_type == LogsLevel.warning:
            ros_logger.warning(msg)
        elif log_type == LogsLevel.error:
            ros_logger.error(msg)
        else:   # LogsLevel.fatal
            ros_logger.fatal(msg)

        if self._file_logger:
            self._file_logger.log(STD_LEVEL[log_type], msg)

    def send_message_with_ack(self, command_payload, retries=DEFAULT_RETRIES, timeout=ACK_TIMEOUT):
        """
        Sends a payload (e.g. "DISPLAY;Battery:100%") as MSG;<id>;<command_payload>\n
        and waits for ACK;<id>, retransmitting up to `retries` times.

        Returns True only if the message was confirmed with an ACK. A NACK (an
        explicit rejection from the firmware, e.g. NO_BUFFER) returns False and is
        not retransmitted: the cause is a lack of resources on the other side rather
        than a loss on the link, so resending immediately tends to be refused again.
        This is the same rule the firmware's send_msg_with_ack_esp follows.
        """
        msg_id = str(self._generate_msg_id())
        full = f"MSG;{msg_id};{command_payload}\n"
        self.generate_log_msg(
            f"Sending msg id={msg_id} payload={command_payload} retries={retries} timeout={timeout}",
            LogsLevel.debug)
        pending = PendingAck()  # carries the wait Event and the reply status
        with self.pending_lock:
            # register it so the reader thread can signal us when the reply arrives
            self.pending_acks[msg_id] = pending

        attempt = 0
        success = False
        while attempt < retries and not self._stop_event.is_set():
            attempt += 1
            try:
                self.ser.write(full.encode())
                self.ser.flush()   # force an immediate send
                self.generate_log_msg(f"Attempt {attempt} sent: {full.strip()}", LogsLevel.debug)
            except Exception as e:
                self.generate_log_msg(f"Error writing to the serial port: {e}", LogsLevel.error)
                break

            # Wait for the reader thread to signal the Event (on ACK;<id> or
            # NACK;<id>;<reason>). wait() returns True if signalled, False on timeout.
            answered = pending.event.wait(timeout)

            if answered and pending.status == PendingAck.ACK:
                self.generate_log_msg(f"ACK confirmed for id={msg_id} on attempt {attempt}", LogsLevel.debug)
                success = True
                break

            if answered and pending.status == PendingAck.NACK:
                # explicit rejection: do not retransmit (see the docstring)
                self.generate_log_msg(
                    f"NACK for id={msg_id} (reason={pending.reason}) on attempt {attempt}: message refused, not retransmitting",
                    LogsLevel.warning)
                success = False
                break

            # timeout: no reply arrived within the deadline
            self.generate_log_msg(
                f"Timeout waiting for ACK id={msg_id} (attempt {attempt}/{retries})", LogsLevel.warning)
            with self.pending_lock:
                self.pending_acks.pop(msg_id, None)   # clear the stale entry
            # If attempts remain, register a fresh entry. This prevents an old Event,
            # possibly already signalled by a late reply, from ending the next wait
            # by mistake.
            if attempt < retries:
                pending = PendingAck()
                with self.pending_lock:
                    self.pending_acks[msg_id] = pending

        # remove whatever is left for this id, whatever the outcome
        with self.pending_lock:
            self.pending_acks.pop(msg_id, None)

        return success

    def set_led(self, color="Branco", retries=DEFAULT_RETRIES, timeout=ACK_TIMEOUT):
        """
        Sets the LED strip to a colour by name. The names are protocol values kept in
        the original language: Branco (white), Laranja (orange), Amarelo (yellow),
        Azul (blue), Verde (green), Roxo (purple), Ciano (cyan), Vermelho (red).
        Any other value turns the strip off.
        """
        payload = f"LED;{color}"
        return self.send_message_with_ack(payload, retries=retries, timeout=timeout)

    def display_text(self, text, retries=DEFAULT_RETRIES, timeout=ACK_TIMEOUT):
        """
        Shows text on the OLED, as "<line_1>;<line_2>".
        Note: avoid very long strings without adjusting the firmware first.
        """
        payload = f"DISPLAY;{text}"
        return self.send_message_with_ack(payload, retries=retries, timeout=timeout)

    def set_pin_out(self, text, retries=DEFAULT_RETRIES, timeout=ACK_TIMEOUT):
        """
        Asks the microcontroller to change the state of an output GPIO.
        Note: the value sent is NOT the physical GPIO number; it is a zero-based
        index into the firmware's PINS_OUT[] array.
        """
        payload = f"PINOUT;{text}"
        return self.send_message_with_ack(payload, retries=retries, timeout=timeout)

    # ROS subscriber callback for /LEDs
    def _led_callback(self, ros_msg):
        """Called when a message arrives on /LEDs (std_msgs/String)."""
        self.generate_log_msg(f"Received on /LEDs: {ros_msg.data}", LogsLevel.debug)
        try:
            ok = self.set_led(ros_msg.data, retries=DEFAULT_RETRIES, timeout=ACK_TIMEOUT)
            if ok:
                self.generate_log_msg(f"LED command sent and acknowledged for colour {ros_msg.data}", LogsLevel.debug)
            else:
                self.generate_log_msg(f"LED command sent BUT not acknowledged for colour {ros_msg.data}", LogsLevel.warning)
        except Exception as e:
            self.generate_log_msg(f"Error {e} while sending the LED command: {ros_msg.data}", LogsLevel.error)

    # ROS subscriber callback for /desliga_hardware
    def _general_off_sub_callback(self, ros_msg):
        """Called when a message arrives on /desliga_hardware (std_msgs/Bool)."""
        self.generate_log_msg(f"Received on /desliga_hardware: {ros_msg.data}", LogsLevel.info)
        if ros_msg.data:
            try:
                ok = self.send_message_with_ack("POWER_OFF", retries=DEFAULT_RETRIES, timeout=ACK_TIMEOUT)
                if ok:
                    self.generate_log_msg("POWER_OFF command sent and acknowledged.", LogsLevel.debug)
                else:
                    self.generate_log_msg("POWER_OFF command sent BUT not acknowledged.", LogsLevel.warning)
            except Exception as e:
                self.generate_log_msg(f"Error sending the POWER_OFF command: {e}", LogsLevel.error)

    # ROS subscriber callback for /oled
    def _display_sub_callback(self, ros_msg):
        """Called when a message arrives on /oled (std_msgs/String)."""
        self.generate_log_msg(f"Received on /oled: {ros_msg.data}", LogsLevel.debug)
        try:
            ok = self.display_text(ros_msg.data, retries=DEFAULT_RETRIES, timeout=ACK_TIMEOUT)
            if ok:
                self.generate_log_msg(f"DISPLAY command sent and acknowledged for message {ros_msg.data}", LogsLevel.debug)
            else:
                self.generate_log_msg(f"DISPLAY command sent BUT not acknowledged for message {ros_msg.data}", LogsLevel.warning)
        except Exception as e:
            self.generate_log_msg(f"Error sending the DISPLAY command: {e}", LogsLevel.error)

    # ROS subscriber callback for /pin_out
    def _pin_out_sub_callback(self, ros_msg):
        """Called when a message arrives on /pin_out (std_msgs/String).

        The payload looks like "0;high" or "1;low": a zero-based index into the
        firmware's PINS_OUT[] array, not a physical GPIO number.
        """
        self.generate_log_msg(f"Received on /pin_out: {ros_msg.data}", LogsLevel.debug)
        try:
            ok = self.set_pin_out(ros_msg.data, retries=DEFAULT_RETRIES, timeout=ACK_TIMEOUT)
            if ok:
                self.generate_log_msg(f"PINOUT command sent and acknowledged for message {ros_msg.data}", LogsLevel.debug)
            else:
                self.generate_log_msg(f"PINOUT command sent BUT not acknowledged for message {ros_msg.data}", LogsLevel.warning)
        except Exception as e:
            self.generate_log_msg(f"Error sending the PINOUT command: {e}", LogsLevel.error)

    # graceful shutdown
    def destroy(self):
        # release the coupling (PINOUT_0)
        try:
            ok = self.set_pin_out("0;high", retries=DEFAULT_RETRIES, timeout=ACK_TIMEOUT)
            if ok:
                self.generate_log_msg("PINOUT command (release coupling) sent and acknowledged.", LogsLevel.debug)
            else:
                self.generate_log_msg("PINOUT command (release coupling) sent BUT not acknowledged.", LogsLevel.warning)
        except Exception as e:
            self.generate_log_msg(f"Error sending the PINOUT command (release coupling): {e}", LogsLevel.error)

        # turn the LED strip off
        try:
            ok = self.set_led("Apagar", retries=DEFAULT_RETRIES, timeout=ACK_TIMEOUT)
            if ok:
                self.generate_log_msg("LED command sent and acknowledged (strip off).", LogsLevel.debug)
            else:
                self.generate_log_msg("LED command sent BUT not acknowledged (strip off).", LogsLevel.info)
        except Exception as e:
            self.generate_log_msg(f"Error sending the LED command (strip off): {e}", LogsLevel.error)

        # clear the display
        try:
            ok = self.display_text(" ;Shutting down..", retries=DEFAULT_RETRIES, timeout=ACK_TIMEOUT)
            if ok:
                self.generate_log_msg("DISPLAY clear command sent and acknowledged.", LogsLevel.debug)
            else:
                self.generate_log_msg("DISPLAY clear command sent BUT not acknowledged.", LogsLevel.warning)
        except Exception as e:
            self.generate_log_msg(f"Error sending the DISPLAY command: {e}", LogsLevel.error)

        # tell the worker threads to stop
        self._stop_event.set()
        # wait for them to finish
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        if self._heartbeat_thread.is_alive():
            self._heartbeat_thread.join(timeout=1.0)
        # close the serial port
        try:
            self.ser.close()
        except Exception:
            pass
        # destroy the node
        try:
            super().destroy_node()
        except Exception:
            pass
        self.generate_log_msg("serial_comm_node finished", LogsLevel.info)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SerialCommNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.generate_log_msg("KeyboardInterrupt received, shutting down...", LogsLevel.info)
    finally:
        try:
            if node:
                node.destroy()
        except Exception:
            pass

        # defensive rclpy shutdown: ignore a double call
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
