/*
  esp32.ino

  - No dynamic allocation (fixed buffer pool).
  - serialTask: reads the serial port continuously, replies with ACK/NACK immediately
                when applicable, and hands the filled buffer index to rxQueue so that
                loop() can process it.
  - heartbeatTask: watches last_ping_ms and trips the emergency stop if it exceeds 1 s.
  - buttonTask: polls the buttons with debounce and sends events to the host (with ACK).
  - ledStripTask: drives the LED strip colour.
  - loop(): consumes rxQueue and processes messages (handle_MSG_command).

  Protocol expected from ROS:
  - Heartbeat: "PING"                  (no ACK; restarts the watchdog timer)
  - Hardware shutdown:                 "MSG;<id>;POWER_OFF"  -> serialTask replies "ACK;<id>\n" on reception
  - LED strip colour:                  "MSG;<id>;LED;<color>" -> serialTask replies "ACK;<id>\n" on reception
  - OLED display text:                 "MSG;<id>;DISPLAY;<line_1>;<line_2>" -> serialTask replies "ACK;<id>\n" on reception
  - Change an output pin of the interface board: "MSG;<id>;PINOUT;<pin>;<high/low>"
        (<pin> is a zero-based index, NOT the microcontroller GPIO number; state is high or low)

  Protocol sent to ROS:
  - Button state:                      "MSG;<id>;BUTTON;<pin>"  (<pin> is a zero-based index, not the GPIO number)
  - Input pin state change:            "MSG;<id>;PININ;<pin_name>;<high/low>"
        (<pin_name> is the name of the #define constant; state is high or low)
  - Acknowledgement of a received message: "ACK;<id>"
  - Buffer/queue unavailable:          "NACK;<id>;REASON"
*/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#ifdef _AVR_
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Fonts/FreeSansBold12pt7b.h>
// #include <Fonts/FreeSans12pt7b.h>

// Turns a #define into a string literal at compile time
#define STRINGIFY(x) #x

// ################ output pins ############################
// control outputs
#define CONTROL_STOP 27  // motor driver emergency stop (active low)
#define CONTROL_OFF 12   // main power switch, used to shut everything down (active low)
// --- grouped for easy iteration ---
const int PINS_CTRL[] = {CONTROL_STOP, CONTROL_OFF};
const int PINS_CTRL_COUNT = sizeof(PINS_CTRL) / sizeof(PINS_CTRL[0]);

#define LED_STRIP_PIN 4
// -------- OLED ------------
#define SDA_OLED 21
#define SCL_OLED 22

// general purpose outputs
#define PINOUT_0 0  // (OUT_1) drives the coupling relay
#define PINOUT_1 9  // dock station coupling relay (interface board: STP_SIG) -> old board (2 MB: gpio 16) | new board (4 MB: gpio 9)
#define PINOUT_2 10 // GIRO_FLEX (interface board: STP_DIR) -> old board (2 MB: gpio 17) | new board (4 MB: gpio 10)

// --- output gpios (grouped for easy iteration) ---
const int PINS_OUT[] = {PINOUT_0, PINOUT_1, PINOUT_2};
const int PINS_OUT_COUNT = sizeof(PINS_OUT) / sizeof(PINS_OUT[0]);
// ##############################################################

// ----------------------------------------------------------------

// ################ input pins ############################
// --- pin array used by buttonTask (buttons only) ---
// The gpios in 'BUTTON_PINS' only trigger on a falling edge
#define BUTTON_0 25   // button that shuts the hardware down
#define BUTTON_1 32
#define BUTTON_2 33
#define BUTTON_3 2
#define BUTTON_4 34
const int BUTTON_PINS[] = {BUTTON_0, BUTTON_1, BUTTON_2, BUTTON_3, BUTTON_4};
const int BUTTON_COUNT = sizeof(BUTTON_PINS) / sizeof(BUTTON_PINS[0]);

// --- pin array for general input monitoring (any transition) ---
// #define IN_0 XX   // battery charger connected
// #define IN_1 XX   // dock station connected
#define IN_2 35   // (interface board: IN_2) limit switch telling whether the coupling is closed
const int MONITOR_PINS[] = {IN_2};
const int MONITOR_COUNT = sizeof(MONITOR_PINS) / sizeof(MONITOR_PINS[0]);
// Literal names of the #defines matching each pin (used in the payload)
const char* const MONITOR_PIN_NAMES[] = {STRINGIFY(IN_2)};

// Buffer size for a "PININ;<name>;<high|low>" payload:
//   "PININ;" (6) + name + ";" (1) + "high" (4) + '\0' (1)  =  12 + strlen(name)
// 48 leaves room for a 36-character #define name. If you add a pin whose name is
// longer than that, build_pinin_payload() reports it instead of silently sending a
// truncated, malformed frame.
const size_t PININ_PAYLOAD_LEN = 48;
// ##############################################################


//------------------ GLOBAL VARIABLES --------------
bool flagStarting = true;

// used to trip the emergency stop when the heartbeat is lost (1 second)
bool flagStopOn = false;
bool flagStopOff = false;

char currentColor[10] = "Branco";   // protocol value, kept in the original language
bool outputsTested = false;
unsigned long start_time = 0;
int shutdownDelaySec = 30;   // countdown, in seconds, before cutting power

// debounce configuration (used by buttonTask)
const unsigned long DEBOUNCE_DELAY_MS = 50;

#define NUMPIXELS 27 // number of segments on the LED strip
Adafruit_NeoPixel pixels(NUMPIXELS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

const uint8_t OLED_ADDR = 0x3C; // display I2C address
bool display_ok = false;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- Serial / Heartbeat / Pool ----------------
const unsigned long HEARTBEAT_TIMEOUT_MS = 1000UL; // milliseconds
volatile unsigned long last_ping_ms = 0; // updated by serialTask

// Pool & queues
const size_t RX_LINE_MAX_LEN = 256; // maximum length of a line, including '\0'; may be reduced to fit the application
const int POOL_SIZE = 10;           // number of buffers in the pool (tune as needed)
const int RX_QUEUE_LENGTH = POOL_SIZE; // capacity of the receive queue
const int FREE_QUEUE_LENGTH = POOL_SIZE;

static char pool[POOL_SIZE][RX_LINE_MAX_LEN]; // fixed buffer pool
// scratch buffer used by serialTask while assembling a line
static char lineBuf[RX_LINE_MAX_LEN];

QueueHandle_t rxQueue = NULL;   // queue holding indices (int) of filled buffers
QueueHandle_t freeQueue = NULL; // queue holding indices (int) of free buffers

// ---------------- Pending ACKs (for messages the ESP sent and is waiting on) ----------
typedef struct {
  bool used;
  unsigned long id;
  SemaphoreHandle_t sem; // binary
  int status; // 0 = pending, 1 = ACK, 2 = NACK
} PendingAck;

static PendingAck pendingAcks[POOL_SIZE]; // limited capacity (POOL_SIZE)

// ---------------- Deduplication of received ids ----------------
// The sender resends the SAME line (same id) when the ACK does not come back in
// time. If the ACK was lost on the return path the message has already been
// executed, and running it again would be wrong. This window keeps the last
// processed ids so a retransmission can be recognised: it is RE-ACKNOWLEDGED
// (that ACK is precisely the one the sender never got) but not enqueued again.
// Touched only by serialTask, so it needs no lock.
const int SEEN_IDS_SIZE = 16;
const unsigned long SEEN_ID_EMPTY = 0xFFFFFFFFUL;   // sentinel: 0 is a valid id
static unsigned long seenIds[SEEN_IDS_SIZE];
static int seenIdsHead = 0;

// ---------------- Serial mutex ----------------
// Protects writes to the UART. A protocol line is assembled from several Serial.*
// calls, and the tasks run on different cores (genuinely parallel writes) — without
// the mutex a line can be split in half by another task's output, and the host
// receives garbage in place of the ACK/NACK/MSG.
SemaphoreHandle_t serialMutex = NULL;

//--- used to inspect the real stack usage of each task
TaskHandle_t serialTaskHandle = NULL;
TaskHandle_t heartbeatTaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t ledStripTaskHandle = NULL;


// ---------------- Forward declarations ----------------
void serialTask(void *pvParameters);
void heartbeatTask(void *pvParameters);
void buttonTask(void *pvParameters);
void ledStripTask(void *pvParameters);
void handle_MSG_command(const char *id, const char *command, const char *args);
void set_logical_gpio(int logical_id, const char *stateButton);
static inline void sendNACK_fromISRContextSafe(const char *id, const char *reason);
unsigned long int generate_msg_id(void);
static inline void sendACK_fromISRContextSafe(const char *id);
void robotStarting(void);
void ledStrip(const char *color);
void display_text(const char *line1, const char *line2);
void runShutdownCountdown(int seconds);
// pendingAcks helpers
static int register_pending_ack(unsigned long id);
static void unregister_pending_ack(int idx);
static void signal_pending_ack(unsigned long id, int status);
// deduplication of received ids
static bool id_already_processed(unsigned long id);
static void register_seen_id(unsigned long id);
// PININ payload assembly (guards against a pin name too long for the buffer)
static bool build_pinin_payload(char *out, size_t out_len, const char *pin_name, int state);

// serialised writes to the UART (see serialMutex)
void serialSendLine(const char *line);
void serialSendLinef(const char *fmt, ...);

// send-with-ACK API (used by the tasks when an event has to reach the host)
bool send_msg_with_ack_esp(const char *payload, int retries = 3, unsigned long timeout_ms = 1000);


// ---------------- Serialised writes to the UART ----------------
/**
   serialSendLine / serialSendLinef
   - Emit ONE complete line at a time, protected by serialMutex.
   - Without this, a protocol line (e.g. "ACK;<id>") can be split in half by
     another task's output and the host receives garbage in place of the ACK.
   - Do NOT use from an ISR: a FreeRTOS mutex cannot be taken inside an
     interrupt. To log something detected in an ISR, raise a flag and let the
     corresponding task emit the line.
   - The two are deliberately independent: the mutex is not recursive, so
     serialSendLinef must not call serialSendLine (that would deadlock).
*/
void serialSendLine(const char *line) {
  if (serialMutex) xSemaphoreTake(serialMutex, portMAX_DELAY);
  Serial.println(line);
  Serial.flush();
  if (serialMutex) xSemaphoreGive(serialMutex);
}

void serialSendLinef(const char *fmt, ...) {
  // Static buffer: keeps the 256-byte line off the caller's stack, and access is
  // already serialised by the mutex.
  //
  // It does NOT make this function cheap on stack. vsnprintf() below is newlib's
  // full formatter and needs roughly 1.3 KB of stack on its own — far more than the
  // buffer this static allocation saves. Any task that reaches this function must be
  // created with at least 4096 bytes; see the sizing note in setup().
  static char lineOutBuf[RX_LINE_MAX_LEN];
  if (serialMutex) xSemaphoreTake(serialMutex, portMAX_DELAY);
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(lineOutBuf, sizeof(lineOutBuf), fmt, ap);
  va_end(ap);
  Serial.println(lineOutBuf);
  Serial.flush();
  if (serialMutex) xSemaphoreGive(serialMutex);
}

void setPins(void) {
  // configure control outputs
  pinMode(CONTROL_STOP, OUTPUT);
  digitalWrite(CONTROL_STOP, HIGH);

  pinMode(CONTROL_OFF, OUTPUT);
  digitalWrite(CONTROL_OFF, HIGH);

  pinMode(LED_STRIP_PIN, OUTPUT);
  digitalWrite(LED_STRIP_PIN, LOW);

  // configure input gpios
  for (int i = 0; i < BUTTON_COUNT; ++i) {
    pinMode(BUTTON_PINS[i], INPUT);
  }
  for (int i = 0; i < MONITOR_COUNT; ++i) {
    pinMode(MONITOR_PINS[i], INPUT);
  }

  // configure general purpose output gpios
  for (int i = 0; i < PINS_OUT_COUNT; ++i) {
    pinMode(PINS_OUT[i], OUTPUT);
    digitalWrite(PINS_OUT[i], LOW);
  }
}

void configDisplayOled() {
  // initialise I2C explicitly (optional — Wire.begin() with no arguments uses
  // the default pins 21 (SDA) and 22 (SCL) on most ESP32 boards)
  Wire.begin(SDA_OLED, SCL_OLED);
  delay(100); // let the display settle after power-up

  display_ok = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);

  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() != 0) {
    serialSendLine("ERROR: lost communication with the OLED display!");
    display_ok = false; // mark as disconnected so we do not keep retrying
  }

  display_text("", "Starting...");
}

void configLedStrip() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(_AVR_ATtiny85_) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'
  delay(100);
}

void setup() {
  // pins
  setPins();

  Serial.begin(115200);
  while (!Serial);
  serialMutex = xSemaphoreCreateMutex();   // must exist before any task or log

  configLedStrip();

  // initialise the display
  configDisplayOled();

  // create the queues
  // RX_QUEUE_LENGTH: maximum number of items the queue can hold at once.
  // sizeof(int): size of each item stored in the queue.
  // The call returns a handle (QueueHandle_t) used later to push items
  // (xQueueSend), pop items (xQueueReceive) and inspect state.
  rxQueue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(int));
  freeQueue = xQueueCreate(FREE_QUEUE_LENGTH, sizeof(int));
  if (rxQueue == NULL || freeQueue == NULL) {
    serialSendLine("ERROR: could not create queues");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // fill freeQueue with indices 0..POOL_SIZE-1
  for (int i = 0; i < POOL_SIZE; ++i) {
    pendingAcks[i].used = false;
    pendingAcks[i].sem = NULL;
    int idx = i;
    xQueueSend(freeQueue, &idx, 0);
  }

  // The deduplication window starts empty. It is filled with the sentinel because
  // the array is zero-initialised and 0 is a valid id (the host counter wraps
  // through 0) — without this, the first message with id 0 would look like a duplicate.
  for (int i = 0; i < SEEN_IDS_SIZE; ++i) {
    seenIds[i] = SEEN_ID_EMPTY;
  }

  start_time = millis();

  ledStrip(currentColor);
  delay(500);
  ledStrip(currentColor);
  delay(500);
  ledStrip(currentColor);
  delay(500);

  // create the tasks
  // xTaskCreatePinnedToCore creates a FreeRTOS task and pins it to one of the two
  // ESP32 cores. loop() runs on core 1, and the peripherals are pushed to core 0.
  // Sizing note: any task that reaches serialSendLinef() pays for newlib's
  // vsnprintf(), which needs roughly 1.3 KB of stack on its own. Give such a task
  // 4096 bytes — below that it overruns the stack canary and the core panics.
  xTaskCreatePinnedToCore(serialTask, "serialTask", 4096, NULL, 3, &serialTaskHandle, 1);
  // serialTask: pointer to the task function.
  // "serialTask": descriptive name.
  // 4096: stack size in BYTES (in the ESP-IDF FreeRTOS port this parameter is in
  //       bytes, unlike vanilla FreeRTOS, which uses words).
  // 3: task priority (higher means more important).
  // &serialTaskHandle: handle used later to inspect or control the task.
  // 1: core the task runs on (0 -> core 0, 1 -> core 1, tskNO_AFFINITY -> either).

  // 4096: calls set_logical_gpio() -> serialSendLinef() -> vsnprintf() when the
  //       watchdog trips, which is precisely the path that must not crash.
  xTaskCreatePinnedToCore(heartbeatTask, "heartbeatTask", 4096, NULL, 1, &heartbeatTaskHandle, 1);
  xTaskCreatePinnedToCore(buttonTask, "buttonTask", 3072, NULL, 1, &buttonTaskHandle, 0);
  // 1536 is enough here: ledStripTask only drives pixels, it never formats text.
  xTaskCreatePinnedToCore(ledStripTask, "ledStripTask", 1536, NULL, 2, &ledStripTaskHandle, 1);

  // initialise the heartbeat timestamp
  last_ping_ms = millis();
}

/**
   loop
   - consumes rxQueue (indices) and processes pool[idx]
   - returns idx to freeQueue once done
   - any remaining application processing can live here
*/
void loop() {
  //-------------------------------- STACK USAGE INSPECTION --------------------------------------------------
  // uxTaskGetStackHighWaterMark() returns the smallest number of stack bytes that
  // remained free since the task started. The lower the number, the closer you got
  // to a stack overflow — increase the task stack size.

  // exercise every output once at boot
  if (outputsTested == false) {
    outputsTested = true;
    for (int i = 0; i < PINS_OUT_COUNT; ++i) {
      set_logical_gpio(PINS_OUT[i], "high");
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
    for (int i = 0; i < PINS_OUT_COUNT; ++i) {
      set_logical_gpio(PINS_OUT[i], "low");
    }
  }

  static unsigned long last_check_time = 0;
  if (millis() - last_check_time > 5000) { // every 5 seconds
    last_check_time = millis();

    UBaseType_t serial_hwm = uxTaskGetStackHighWaterMark(serialTaskHandle);
    UBaseType_t heartbeat_hwm = uxTaskGetStackHighWaterMark(heartbeatTaskHandle);
    UBaseType_t button_hwm = uxTaskGetStackHighWaterMark(buttonTaskHandle);
    UBaseType_t ledStrip_hwm = uxTaskGetStackHighWaterMark(ledStripTaskHandle);

    // whole block under the mutex: these 6 lines must come out uninterrupted
    if (serialMutex) xSemaphoreTake(serialMutex, portMAX_DELAY);
    Serial.println("--- Stack High Water Mark (free bytes) ---");
    Serial.printf("serialTask: %u\n", serial_hwm);
    Serial.printf("heartbeatTask: %u\n", heartbeat_hwm);
    Serial.printf("buttonTask: %u\n", button_hwm);
    Serial.printf("ledStripTask: %u\n", ledStrip_hwm);
    Serial.println("------------------------------------------");
    Serial.flush();
    if (serialMutex) xSemaphoreGive(serialMutex);

  }
  //---------------------------------------------------------------------------------------------------------------------------------


  int bufIndex;
  // Try to take an item from the inbox (rxQueue), waiting at most 100 ms.
  // While blocked, other tasks run. pdMS_TO_TICKS(100) converts milliseconds into
  // FreeRTOS ticks (the scheduler's internal unit).
  if (xQueueReceive(rxQueue, &bufIndex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // xQueueReceive returns pdTRUE if an item was received before the timeout,
    // pdFALSE if the 100 ms elapsed with an empty queue.

    // process the message stored in pool[bufIndex]
    char *msg = pool[bufIndex];

    serialSendLinef("LOOP: message received: %s", msg);

    // parse the message (MSG;id;COMMAND;ARGS)
    // note: the ACK was already sent by serialTask on reception
    if (strncmp(msg, "MSG;", 4) == 0) {
      const char *p = msg + 4;    // p points just past "MSG;"
      const char *colon = strchr(p, ';'); // first ';' after the id
      if (!colon) {
        serialSendLine("ERROR:MSG_FORMAT");
      }
      else {
        // extract the id
        char idbuf[32];
        size_t idlen = colon - p;
        if (idlen >= sizeof(idbuf)) idlen = sizeof(idbuf) - 1;
        memcpy(idbuf, p, idlen);
        idbuf[idlen] = '\0';

        const char *rest = colon + 1;   // everything after the id
        const char *second_colon = strchr(rest, ';');   // separates the command from its arguments
        if (!second_colon) {    // no ';' after the command means there are no arguments
          char commandbuf[20];
          size_t cmdlen = strlen(rest);
          if (cmdlen >= sizeof(commandbuf)) cmdlen = sizeof(commandbuf) - 1;
          memcpy(commandbuf, rest, cmdlen);
          commandbuf[cmdlen] = '\0';
          handle_MSG_command(idbuf, commandbuf, "");
        }
        else {  // second_colon present: the message carries arguments
          char commandbuf[82];
          size_t cmdlen = second_colon - rest;
          if (cmdlen >= sizeof(commandbuf)) cmdlen = sizeof(commandbuf) - 1;
          memcpy(commandbuf, rest, cmdlen);
          commandbuf[cmdlen] = '\0';
          const char *args = second_colon + 1;
          handle_MSG_command(idbuf, commandbuf, args);
        }
      }
    }
    else {
      // non-MSG lines
      serialSendLinef("LOOP: message received outside the protocol format: %s", msg);
    }

    // return the buffer to the pool (freeQueue)
    int idxToReturn = bufIndex;
    xQueueSend(freeQueue, &idxToReturn, 0);
  }

  vTaskDelay(pdMS_TO_TICKS(50));  // yield the CPU so equal or lower priority tasks can run
}

void robotStarting() {
  pixels.clear();
  for (uint8_t i = 0; i < NUMPIXELS; i++) { // For each pixel...
    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(255, 255, 255));
  }

  for (uint8_t i = 1; i < 255; i++) { // fade in
    pixels.setBrightness(i);
    pixels.show();
  }

  for (uint8_t i = 255; i > 0; i--) { // fade out
    pixels.setBrightness(i);
    pixels.show();
  }
  pixels.clear();

}

// ---------------- ACK/NACK helpers (thread-safe) ----------------
/**
   sendACK - emits "ACK;<id>\n"
   used by serialTask as soon as a MSG;<id>;... line is recognised
*/
static inline void sendACK_fromISRContextSafe(const char *id) {
  // we are not in an ISR here; the name is kept to signal direct, low-latency use
  serialSendLinef("ACK;%s", id);
}

unsigned long int generate_msg_id(void) {
  // Short incrementing id (avoids a heavyweight uuid).
  static unsigned long int idCont = 0;
  idCont++;
  if (idCont >= 4294967294) {
    idCont = 0;
  }
  return idCont;
}

/**
   sendNACK - emits "NACK;<id>;<reason>\n"
*/
static inline void sendNACK_fromISRContextSafe(const char *id, const char *reason) {
  serialSendLinef("NACK;%s;%s", id, reason);
}

// ---------------- pendingAcks implementation ----------------
static int register_pending_ack(unsigned long id) {
  for (int i = 0; i < POOL_SIZE; ++i) {
    if (!pendingAcks[i].used) {
      pendingAcks[i].used = true;
      pendingAcks[i].id = id;
      pendingAcks[i].status = 0;
      pendingAcks[i].sem = xSemaphoreCreateBinary();
      if (pendingAcks[i].sem == NULL) {
        // could not create the semaphore: release the slot and fail
        pendingAcks[i].used = false;
        return -1;
      }
      return i;
    }
  }
  return -1; // no free slot
}

static void unregister_pending_ack(int idx) {
  if (idx < 0 || idx >= POOL_SIZE) return;
  if (pendingAcks[idx].used) {
    if (pendingAcks[idx].sem) {
      vSemaphoreDelete(pendingAcks[idx].sem);
    }
    pendingAcks[idx].used = false;
    pendingAcks[idx].id = 0;
    pendingAcks[idx].sem = NULL;
    pendingAcks[idx].status = 0;
  }
}

static void signal_pending_ack(unsigned long id, int status) {
  for (int i = 0; i < POOL_SIZE; ++i) {
    if (pendingAcks[i].used && pendingAcks[i].id == id) {
      pendingAcks[i].status = status; // 1 = ACK, 2 = NACK
      if (pendingAcks[i].sem) {
        xSemaphoreGive(pendingAcks[i].sem);
      }
      break;
    }
  }
}

// ---------------- Deduplication implementation ----------------
/**
   id_already_processed - true if the id is still inside the received-ids window.
   A linear scan over 16 entries: negligible next to the cost of the UART.
*/
static bool id_already_processed(unsigned long id) {
  for (int i = 0; i < SEEN_IDS_SIZE; ++i) {
    if (seenIds[i] == id) return true;
  }
  return false;
}

/**
   register_seen_id - stores the id in the circular window, dropping the oldest one.
   Called by serialTask as soon as the message enters rxQueue — never after it is
   executed in loop(), which runs asynchronously and would leave a window in which
   a fast retransmission could slip past the check.
*/
static void register_seen_id(unsigned long id) {
  seenIds[seenIdsHead] = id;
  seenIdsHead = (seenIdsHead + 1) % SEEN_IDS_SIZE;
}


// ---------------- PININ payload assembly ----------------
/**
   build_pinin_payload - assembles "PININ;<name>;<high|low>" into out.

   snprintf() truncates silently when the buffer is too small, which would put a
   malformed frame on the wire: the host would parse "PININ;IN_LONG_NAM" with no
   state field and drop it as malformed, so a real pin transition would be lost
   with nothing pointing at the cause. Checking the return value turns that into an
   explicit diagnostic line instead.

   @return true if the payload was assembled in full; false if the name did not fit.
*/
static bool build_pinin_payload(char *out, size_t out_len, const char *pin_name, int state) {
  int n = snprintf(out, out_len, "PININ;%s;%s", pin_name, state == HIGH ? "high" : "low");
  if (n < 0 || (size_t)n >= out_len) {
    // n is what snprintf WOULD have written, so n >= out_len means it was truncated
    serialSendLinef("ERROR:PININ_NAME_TOO_LONG:%s", pin_name);
    return false;
  }
  return true;
}

// ---------------- Send with ACK ----------------
bool send_msg_with_ack_esp(const char *payload, int retries, unsigned long timeout_ms) {
  unsigned long id = generate_msg_id();
  char full[RX_LINE_MAX_LEN];
  // assemble: MSG;<id>;<payload>   (no '\n': serialSendLine uses println)
  snprintf(full, sizeof(full), "MSG;%lu;%s", id, payload);

  int slot = register_pending_ack(id);
  if (slot < 0) {
    // no resource left to wait for the ACK: do not even try to send
    serialSendLine("WARN:NO_PENDING_SLOT");
    return false;
  }

  bool success = false;
  for (int attempt = 1; attempt < (retries + 1); ++attempt) {
    serialSendLine(full);

    // wait with timeout
    if (xSemaphoreTake(pendingAcks[slot].sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
      // we were signalled; check the status
      if (pendingAcks[slot].status == 1) {
        success = true;
        break;
      } else if (pendingAcks[slot].status == 2) {
        // explicit NACK from the host: no automatic retry, the cause is a lack of
        // resources on the other side rather than a loss on the link
        success = false;
        break;
      } else {
        // spurious signal: treat as a failure and possibly retry
      }
    } else {
      // timeout: retry if attempts remain
    }
  }

  unregister_pending_ack(slot);
  return success;
}

// ---------------- Tasks ----------------

/**
   buttonTask
   - polls the buttons (BUTTON_PINS) with debounce
   - on a confirmed press (HIGH -> LOW), sends "MSG;<id>;BUTTON;<index>"
   - waits for the ACK through send_msg_with_ack_esp
*/
void buttonTask(void *pvParameters) {
  (void)pvParameters;

  // local debounce state (used only by this task)
  int last_read_state[BUTTON_COUNT];          // last raw reading of the pin (may be noise)
  int last_steady_state_local[BUTTON_COUNT];  // last confirmed, stable state (HIGH or LOW)
  uint32_t last_event_tick_ms[BUTTON_COUNT];  // timestamp (ms) of the last observed change

  // seed the state with the current reading
  for (int i = 0; i < BUTTON_COUNT; ++i) {
    last_read_state[i] = digitalRead(BUTTON_PINS[i]);
    last_steady_state_local[i] = last_read_state[i];
    last_event_tick_ms[i] = xTaskGetTickCount() * portTICK_PERIOD_MS;
  }

  // same, for the general input monitoring (detects any transition)
  int last_read_mon[MONITOR_COUNT];
  int last_steady_mon[MONITOR_COUNT];
  uint32_t last_event_mon[MONITOR_COUNT];
  for (int i = 0; i < MONITOR_COUNT; ++i) {
    last_read_mon[i] = digitalRead(MONITOR_PINS[i]);
    last_steady_mon[i] = last_read_mon[i];
    last_event_mon[i] = xTaskGetTickCount() * portTICK_PERIOD_MS;
  }

  // Publish the initial state of every MONITOR_PIN to ROS, 0.5 s apart, so the
  // host never has to assume a starting value.
  for (int i = 0; i < MONITOR_COUNT; ++i) {
    int state = digitalRead(MONITOR_PINS[i]);
    char payload[PININ_PAYLOAD_LEN];
    if (!build_pinin_payload(payload, sizeof(payload), MONITOR_PIN_NAMES[i], state)) {
      continue;   // name too long: already reported, do not send a malformed frame
    }
    serialSendLine(payload);

    bool ok = send_msg_with_ack_esp(payload, 3, 1000);
    if (ok) {
      serialSendLine("INITIAL_PININ_EVENT: ACK received from the host");
    } else {
      serialSendLine("INITIAL_PININ_EVENT: no ACK from the host (timeout)");
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }

  for (;;) {
    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    for (int i = 0; i < BUTTON_COUNT; ++i) {
      int pin = BUTTON_PINS[i];
      int state = digitalRead(pin);

      if (state != last_read_state[i]) {
        // change detected: record the time and restart the debounce window
        last_read_state[i] = state;
        last_event_tick_ms[i] = now_ms;
      }
      else {
        // stable for DEBOUNCE_DELAY_MS: if it differs from the steady state, confirm the transition
        if ((now_ms - last_event_tick_ms[i]) >= DEBOUNCE_DELAY_MS) {
          if (last_read_state[i] != last_steady_state_local[i]) {
            if (last_steady_state_local[i] == HIGH && last_read_state[i] == LOW) {
              // press detected (HIGH -> LOW): build the payload and send it with ACK
              char payload[12];
              snprintf(payload, sizeof(payload), "BUTTON;%d", i);
              serialSendLine(payload);

              if (pin == BUTTON_0) {  // button that shuts the robot down
                bool ok = send_msg_with_ack_esp(payload, 1, 1000);
                if (ok) {
                  serialSendLine("BUTTON_EVENT: ACK received from the host");
                }
                else {
                  for (int attempt = 0; attempt < 3; ++attempt) {    // three more attempts
                    ok = send_msg_with_ack_esp(payload, 1, 2000);    // one send, waiting 2 s
                    if (ok) {
                      serialSendLine("BUTTON_EVENT: ACK received from the host");
                      vTaskDelay(pdMS_TO_TICKS(3000));  // brief pause so subscribers can react
                      break;
                    }
                  }
                  serialSendLine("BUTTON_EVENT: no ACK from the host (timeout)");
                }
                runShutdownCountdown(shutdownDelaySec);

                set_logical_gpio(CONTROL_OFF, "low");
                vTaskDelay(pdMS_TO_TICKS(10000));       // hold the gpio for 10 s so the supply capacitors can discharge
              }
              else {
                bool ok = send_msg_with_ack_esp(payload, 3, 1000);
                if (ok) {
                  serialSendLine("BUTTON_EVENT: ACK received from the host");
                }
                else {
                  serialSendLine("BUTTON_EVENT: no ACK from the host (timeout)");
                }
                serialSendLinef("BUTTON %d pressed", pin);
              }
            }
            // update the steady state
            last_steady_state_local[i] = last_read_state[i];
          }
        }
      }
    }

    // --- Monitored pins (any transition: HIGH->LOW or LOW->HIGH) ---
    // Unlike the buttons (which only fire on a falling edge), this section detects
    // any stable state change. It is used for sensors such as limit switches, where
    // both engaging (LOW->HIGH) and releasing (HIGH->LOW) are relevant events.
    for (int i = 0; i < MONITOR_COUNT; ++i) {
      int pin = MONITOR_PINS[i];
      int state = digitalRead(pin);

      if (state != last_read_mon[i]) {
        // Change detected: update the raw reading and restart the debounce timer.
        // The transition is not confirmed yet — we wait for the stability period.
        last_read_mon[i] = state;
        last_event_mon[i] = now_ms;
      } else {
        // The state has been stable long enough since the last change.
        if ((now_ms - last_event_mon[i]) >= DEBOUNCE_DELAY_MS) {
          // If the stabilised reading differs from the last confirmed state,
          // we have a real transition.
          if (last_read_mon[i] != last_steady_mon[i]) {
            char payload[PININ_PAYLOAD_LEN];
            if (build_pinin_payload(payload, sizeof(payload), MONITOR_PIN_NAMES[i], state)) {
              serialSendLine(payload);

              bool ok = send_msg_with_ack_esp(payload, 3, 1000);
              if (ok) {
                serialSendLine("PININ: ACK received from the host");
              } else {
                serialSendLine("PININ: no ACK from the host (timeout)");
              }
            }
            // The steady state is updated either way: if the name does not fit, the
            // problem is the #define, not the pin, and retrying every 100 ms would
            // only flood the link with the same error line.
            last_steady_mon[i] = last_read_mon[i];
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(100));   // polling rate
  }
}

/**
   serialTask
   - reads bytes from the UART continuously
   - accumulates them until '\n'
   - "PING" -> refreshes last_ping_ms
   - "MSG;<id>;..." -> replies ACK;<id> immediately and tries to enqueue the message
     (uses the buffer pool; if it is full, replies NACK;<id>;NO_BUFFER or NACK;<id>;QUEUE_FULL)
   - any other line -> tries to enqueue it without an ACK
*/
void serialTask(void *pvParameters) {
  (void) pvParameters;  // avoids an unused-parameter warning
  size_t idx = 0; // index used to fill lineBuf byte by byte

  for (;;) {
    while (Serial.available()) {  // drain every byte currently in the receive buffer
      int c = Serial.read();    // first available byte (0..255), or -1 if there is nothing
      if (c == -1) break;       // defensive: available() > 0 should guarantee a byte
      if (c == '\r') continue;  // ignore CR — common when the other side sends \r\n
      if (c == '\n') {          // line terminator: the accumulated line is complete
        lineBuf[idx] = '\0';
        if (idx > 0) {          // skip empty lines produced by a lone '\n'
          // ACK/NACK produced by the host
          if (strncmp(lineBuf, "ACK;", 4) == 0) {
            const char *p = lineBuf + 4;
            unsigned long id = strtoul(p, NULL, 10);
            signal_pending_ack(id, 1);
          }
          else if (strncmp(lineBuf, "NACK;", 5) == 0) {
            const char *p = lineBuf + 5;
            // format: NACK;<id>;REASON
            const char *colon = strchr(p, ';');
            unsigned long id = 0;
            if (colon) {
              char idbuf[32];
              size_t idlen = colon - p;
              if (idlen >= sizeof(idbuf)) idlen = sizeof(idbuf) - 1;
              memcpy(idbuf, p, idlen);
              idbuf[idlen] = '\0';
              id = strtoul(idbuf, NULL, 10);
            }
            else {
              id = strtoul(p, NULL, 10);
            }
            signal_pending_ack(id, 2);
          }
          else if (strcmp(lineBuf, "PING") == 0) {
            last_ping_ms = millis();
            serialSendLine("PING received");
          }
          else { // messages coming from the host: commands or general purpose lines
            if (strncmp(lineBuf, "MSG;", 4) == 0) {
              const char *p = lineBuf + 4;    // start of <id>;...
              const char *colon = strchr(p, ';');   // separator between <id> and the rest
              if (!colon) {
                // invalid format: no id, so we cannot even send a targeted NACK
                serialSendLine("ERROR:MSG_NO_ID");
              }
              else {
                // extract the id
                char idbuf[32];
                size_t idlen = colon - p;
                if (idlen >= sizeof(idbuf)) idlen = sizeof(idbuf) - 1;  // controlled truncation, never an overflow
                memcpy(idbuf, p, idlen);
                idbuf[idlen] = '\0';

                // Send the ACK immediately — including for retransmissions: that is
                // exactly the ACK the sender never received the first time. Dropping
                // a duplicate silently would make the sender exhaust its attempts and
                // report failure for a command that already ran.
                sendACK_fromISRContextSafe(idbuf);

                unsigned long id_num = strtoul(idbuf, NULL, 10);
                if (id_already_processed(id_num)) {
                  // Retransmission caused by a lost ACK: already re-acknowledged above.
                  // Enqueuing it again would execute the same command twice.
                  serialSendLinef("DUP:%s", idbuf);
                }
                else {
                  // take a free index from freeQueue (non-blocking)
                  int bufIndex = -1;
                  if (xQueueReceive(freeQueue, &bufIndex, 0) == pdTRUE) {
                    // copy the whole line into pool[bufIndex], guaranteeing termination
                    strncpy(pool[bufIndex], lineBuf, RX_LINE_MAX_LEN - 1);
                    pool[bufIndex][RX_LINE_MAX_LEN - 1] = '\0';

                    // enqueue the filled buffer index for loop() to process, waiting up to 100 ms
                    if (xQueueSend(rxQueue, &bufIndex, pdMS_TO_TICKS(100)) != pdTRUE) {
                      // could not enqueue: give the buffer back and tell the sender
                      int tmp = bufIndex;
                      xQueueSend(freeQueue, &tmp, 0);
                      sendNACK_fromISRContextSafe(idbuf, "QUEUE_FULL");
                    }
                    else {
                      // Only record what actually made it into the queue: a message that
                      // got a NACK was never processed and, if resent, must be treated as
                      // new. Recording happens here (in serialTask, before loop() runs it)
                      // rather than after execution, because serialTask handles lines
                      // sequentially — by the time a retransmission arrives, this record exists.
                      register_seen_id(id_num);
                    }
                  }
                  else {
                    // No free buffer. The receiver accepted the line (ACK) but has no slot
                    // to store it for later processing, so the sender is told explicitly.
                    sendNACK_fromISRContextSafe(idbuf, "NO_BUFFER");
                  }
                }
              }
            }
            else {
              // Line that does not follow MSG;<id>;: try to enqueue it without an ACK.
              // If there is no free buffer, drop it and log WARN:NO_BUFFER_DROP. Neither
              // ACK nor NACK is sent because there is no id to reference (protocol decision).
              int bufIndex = -1;
              if (xQueueReceive(freeQueue, &bufIndex, 0) == pdTRUE) {
                strncpy(pool[bufIndex], lineBuf, RX_LINE_MAX_LEN - 1);
                pool[bufIndex][RX_LINE_MAX_LEN - 1] = '\0';
                if (xQueueSend(rxQueue, &bufIndex, pdMS_TO_TICKS(100)) != pdTRUE) {
                  int tmp = bufIndex;
                  xQueueSend(freeQueue, &tmp, 0);
                }
              }
              else {
                serialSendLine("WARN:NO_BUFFER_DROP");
              }
            }
          }
        }
        idx = 0;    // restart the buffer for the next line
      }
      else {  // any other character is accumulated into the line buffer
        // If there is no room left, discard the line under construction and report it.
        // This prevents an out-of-bounds write and signals an unexpectedly long line.
        if (idx < RX_LINE_MAX_LEN - 1) {
          lineBuf[idx++] = (char)c;
        } else {
          idx = 0;
          serialSendLine("ERROR:LINEBUF_OVERFLOW");
        }
      }
    } // while Serial.available()

    vTaskDelay(pdMS_TO_TICKS(50)); // sleep 50 ms: avoids busy-waiting and lets the scheduler run other tasks
  } // for
}

/**
   heartbeatTask
   - watches last_ping_ms and trips CONTROL_STOP if it exceeds HEARTBEAT_TIMEOUT_MS
   - runs every 500 ms
*/
void heartbeatTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {

    if ((millis() - last_ping_ms) > HEARTBEAT_TIMEOUT_MS) { // trip the motor driver emergency stop
      if (!flagStopOn) {
        flagStopOn = true;
        flagStopOff = false;
        set_logical_gpio(CONTROL_STOP, "low");
      }
    }
    else {
      if (!flagStopOff) {
        flagStopOff = true;
        flagStopOn = false;
        set_logical_gpio(CONTROL_STOP, "high");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

/**
   ledStripTask
   - drives the LED strip colour from the global state variable
   - runs every 300 ms
*/
void ledStripTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {

    if (flagStarting) {
      robotStarting();
    }
    else {
      ledStrip(currentColor);   // the colour comes from the global 'currentColor'
    }

    vTaskDelay(pdMS_TO_TICKS(300));    // 300 ms: one full pass over the strip (1.35 m) takes 216 ms
  }
}

// ---------------- Command processing ----------------

/**
   handle_MSG_command - executes a command (LED/DISPLAY/POWER_OFF/PINOUT)
   Note: the ACK was already sent on reception; here we only execute and log.
*/
void handle_MSG_command(const char *id, const char *command, const char *args) {

  serialSendLinef("CMD:%s ARGS:%s", command, args);

  if (strcmp(command, "LED") == 0) {
    strncpy(currentColor, args, sizeof(currentColor) - 1);
    currentColor[sizeof(currentColor) - 1] = '\0';

    if (flagStarting) {
      flagStarting = false;
    }
    serialSendLinef("EXEC_OK:LED:%s", args);
    return;
  }

  else if (strcmp(command, "DISPLAY") == 0) {
    // args arrives as "line1;line2"
    const char *p = args;
    const char *colon = strchr(p, ';');
    if (!colon) {
      // single line
      display_text(p, "");
    }
    else {
      char line1[30], line2[30];
      size_t len1 = colon - p;
      if (len1 >= sizeof(line1)) len1 = sizeof(line1) - 1;
      memcpy(line1, p, len1);
      line1[len1] = '\0';
      strncpy(line2, colon + 1, sizeof(line2) - 1);
      line2[sizeof(line2) - 1] = '\0';
      display_text(line1, line2);
    }
    serialSendLinef("EXEC_OK:DISPLAY:%s", args);
    return;
  }

  else if (strcmp(command, "POWER_OFF") == 0) {
    // tell the host to shut down, wait for the ACK, then cut power
    char payload[16];
    snprintf(payload, sizeof(payload), "BUTTON;0");
    serialSendLine(payload);

    bool ok = send_msg_with_ack_esp(payload, 3, 1000);
    if (ok) {
      serialSendLine("EVENT: ACK received from the host");
    }
    else {
      serialSendLine("EVENT: no ACK from the host (timeout)");
    }
    runShutdownCountdown(shutdownDelaySec);

    set_logical_gpio(CONTROL_OFF, "low");
    vTaskDelay(pdMS_TO_TICKS(10000));       // hold the gpio for 10 s so the supply capacitors can discharge
    return;
  }

  else if (strcmp(command, "PINOUT") == 0) {
    // args arrives as "<index>;high|low"
    const char *p = args;
    const char *colon = strchr(p, ';');
    if (!colon) {   // no ';' means no state; bail out before dereferencing colon
      serialSendLinef("EXEC_FAIL:PINOUT_FORMAT:%s", args);
      return;
    }

    char pin_out_number[3], state[5];
    size_t len1 = colon - p;
    if (len1 >= sizeof(pin_out_number)) len1 = sizeof(pin_out_number) - 1;
    memcpy(pin_out_number, p, len1);
    pin_out_number[len1] = '\0';
    strncpy(state, colon + 1, sizeof(state) - 1);
    state[sizeof(state) - 1] = '\0';

    signed short int s = atoi(pin_out_number);
    if (s < 0 || s >= PINS_OUT_COUNT) {   // an out-of-range index would read invalid memory
      serialSendLinef("EXEC_FAIL:PINOUT_RANGE:%s", args);
      return;
    }
    set_logical_gpio(PINS_OUT[s], state);

    serialSendLinef("EXEC_OK:PINOUT:%s", args);
    return;
  }

  serialSendLinef("EXEC_FAIL:UNKNOWN_CMD:%s", command);
}

/**
   set_logical_gpio - changes the state of the GPIO mapped to logical_id
*/
void set_logical_gpio(int logical_id, const char *stateButton) {
  // control pins
  for (int i = 0; i < PINS_CTRL_COUNT; ++i) {
    if (PINS_CTRL[i] == logical_id) {
      if (strcmp(stateButton, "high") == 0) {
        digitalWrite(logical_id, HIGH);
        serialSendLinef("GPIO_SETTED_HIGH:%d", logical_id);
      }
      else if (strcmp(stateButton, "low") == 0) {
        digitalWrite(logical_id, LOW);
        serialSendLinef("GPIO_SETTED_LOW:%d", logical_id);
      }
      else {
        serialSendLine("ERROR: UNKNOWN STATE!");
      }
      return;
    }
  }

  // general purpose pins
  for (int i = 0; i < PINS_OUT_COUNT; ++i) {
    if (PINS_OUT[i] == logical_id) {
      if (strcmp(stateButton, "high") == 0) {
        digitalWrite(logical_id, HIGH);
        serialSendLinef("GPIO_SETTED_HIGH:%d", logical_id);
      }
      else if (strcmp(stateButton, "low") == 0) {
        digitalWrite(logical_id, LOW);
        serialSendLinef("GPIO_SETTED_LOW:%d", logical_id);
      }
      else {
        serialSendLine("ERROR: UNKNOWN STATE!");
      }
      return;
    }
  }

  serialSendLine("ERROR: UNKNOWN GPIO!");
}

/**
   ledStrip - sets the LED strip colour.
   Colour names are protocol values and are kept in the original language:
   Branco (white), Laranja (orange), Amarelo (yellow), Azul (blue), Verde (green),
   Roxo (purple), Ciano (cyan), Vermelho (red). Anything else turns the strip off.
*/
void ledStrip(const char *color) {
  pixels.clear();
  for (uint8_t i = 0; i < NUMPIXELS; i++) { // For each pixel...
    if (strcmp(currentColor, "Branco") == 0) {          // white
      pixels.setPixelColor(i, pixels.Color(255, 255, 255)); // red, blue, green
    }
    else if (strcmp(currentColor, "Laranja") == 0) {    // orange
      pixels.setPixelColor(i, pixels.Color(255, 0, 30));
    }
    else if (strcmp(currentColor, "Amarelo") == 0) {    // yellow
      pixels.setPixelColor(i, pixels.Color(255, 2, 70));
    }
    else if (strcmp(currentColor, "Azul") == 0) {       // blue
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));
    }
    else if (strcmp(currentColor, "Verde") == 0) {      // green
      pixels.setPixelColor(i, pixels.Color(0, 0, 255));
    }
    else if (strcmp(currentColor, "Roxo") == 0) {       // purple
      pixels.setPixelColor(i, pixels.Color(255, 50, 0));
    }
    else if (strcmp(currentColor, "Ciano") == 0) {      // cyan
      pixels.setPixelColor(i, pixels.Color(100, 230, 255));
    }
    else if (strcmp(currentColor, "Vermelho") == 0) {   // red
      pixels.setPixelColor(i, pixels.Color(255, 0, 0));
    }
    else {                                              // unknown name: strip off
      pixels.setPixelColor(i, pixels.Color(0, 0, 0));
    }
    pixels.setBrightness(255);
    pixels.show();   // Send the updated pixel colors to the hardware.

    vTaskDelay(pdMS_TO_TICKS(10));
    pixels.clear();
  }
}


/**
   display_text - shows text on the OLED display.
   Draws two centred lines, each with its own font.
*/
void display_text(const char *line1, const char *line2) {
  Wire.beginTransmission(OLED_ADDR);
  if (Wire.endTransmission() != 0) {
    serialSendLine("ERROR: lost communication with the OLED display!");
    return;
  }

  const GFXfont *font1 = &FreeSansBold12pt7b;
  const GFXfont *font2 = &FreeSansBold12pt7b;
  int spacing = 6;

  display.clearDisplay();
  display.setTextWrap(false);

  // ---- measure line 1 ----
  display.setFont(font1);
  int16_t x1o, y1o;
  uint16_t w1, h1;
  display.getTextBounds(line1, 0, 0, &x1o, &y1o, &w1, &h1);

  // ---- measure line 2 ----
  display.setFont(font2);
  int16_t x2o, y2o;
  uint16_t w2, h2;
  display.getTextBounds(line2, 0, 0, &x2o, &y2o, &w2, &h2);

  // total height of both lines (including the spacing)
  int total_h = (int)h1 + ((line2 && line2[0]) ? (spacing + (int)h2) : 0);

  // vertical origin that centres the block
  int y_top = (SCREEN_HEIGHT - total_h) / 2;
  if (y_top < 0) y_top = 0;

  // horizontal origin that centres each line
  int x_line1 = (SCREEN_WIDTH - (int)w1) / 2;
  if (x_line1 < 0) x_line1 = 0;
  int x_line2 = (SCREEN_WIDTH - (int)w2) / 2;
  if (x_line2 < 0) x_line2 = 0;

  // Cursor Y: y_top + h1 works well for most GFX fonts.
  // If the vertical alignment looks off, nudge it by a few pixels.
  int cursor_y1 = y_top + (int)h1;
  int cursor_y2 = y_top + (int)h1 + spacing + (int)h2;

  // draw line 1
  display.setFont(font1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x_line1, cursor_y1);
  display.print(line1);

  // draw line 2, if any
  if (line2 && line2[0]) {
    display.setFont(font2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(x_line2, cursor_y2);
    display.print(line2);
  }

  display.display();
}

/**
   @brief Runs a shutdown countdown on the OLED display.

   Non-blocking for the rest of the system, but it monopolises the calling task
   for the whole countdown.

   @param seconds Number of seconds to count down.
*/
void runShutdownCountdown(int seconds) {
  serialSendLinef("Shutting down in %d seconds...", seconds);

  for (int i = seconds; i >= 0; i--) {
    char msg[16];
    snprintf(msg, sizeof(msg), "%ds left", i);
    display_text("Shutdown:", msg);

    // one-second pause that still lets the rest of the system run
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  display_text("Robot", "Powered off.");
  vTaskDelay(pdMS_TO_TICKS(1000)); // brief pause so the message stays visible
}
