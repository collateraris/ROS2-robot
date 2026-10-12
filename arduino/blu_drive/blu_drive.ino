// Arduino Uno R3 / classic Nano ATmega328P only.
// REQUIRED WIRING CHANGE: original ENA on D1 conflicts with USB TX. Move ENA
// to PWM D9 and remove the driver's ENA/ENB jumpers if fitted. ENB stays on D6.
// Bluetooth RX D10 <- module TX; TX D11 -> module RX through suitable level
// shifting. Hardware USB Serial and SoftwareSerial both use 9600 baud, 8N1.
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if !defined(__AVR_ATmega328P__) || (!defined(ARDUINO_AVR_UNO) && !defined(ARDUINO_AVR_NANO))
#error "Select Arduino Uno R3 or classic Nano with ATmega328P"
#endif

constexpr uint8_t L1 = 3, L2 = 2, R1 = 4, R2 = 5, ENB = 6, ENA = 9;
constexpr uint8_t BT_RX = 10, BT_TX = 11;
constexpr int ROS_LEFT_SIGN = -1, ROS_RIGHT_SIGN = -1;
// ROS positive forward therefore matches original BT F electrically. Physical
// forward and ROS positive yaw still require a raised-wheel wiring check.
static_assert((ROS_LEFT_SIGN == 1 || ROS_LEFT_SIGN == -1) &&
              (ROS_RIGHT_SIGN == 1 || ROS_RIGHT_SIGN == -1), "ROS signs must be +/-1");
constexpr uint32_t MOTOR_PINS = (1UL << L1) | (1UL << L2) | (1UL << R1) |
    (1UL << R2) | (1UL << ENA) | (1UL << ENB) | (1UL << BT_RX) | (1UL << BT_TX) | (1UL << A0);
static_assert(MOTOR_PINS == (1UL << L1) + (1UL << L2) + (1UL << R1) +
    (1UL << R2) + (1UL << ENA) + (1UL << ENB) + (1UL << BT_RX) + (1UL << BT_TX) + (1UL << A0),
    "Pins must be unique");
static_assert(!(MOTOR_PINS & 3UL), "D0/D1 are reserved for USB Serial");
static_assert(ENA == 9 && ENB == 6, "Enable pins must be hardware PWM D9/D6");

// Legacy apps may send only one F on press and one S on release. The default
// therefore has NO automatic stop on Bluetooth link loss. Enable a timeout ONLY
// after measuring repeated motion commands while held. A timeout/link loss then
// latches BT stopped until S/D. Optional D8 requires a correctly wired HC-05 STATE
// signal that actually means connected; do not infer link status from silence.
#ifndef BT_COMMAND_TIMEOUT_MS
#define BT_COMMAND_TIMEOUT_MS 0
#endif
#ifndef BT_STATE_PIN
#define BT_STATE_PIN -1
#endif
static_assert(BT_STATE_PIN == -1 || BT_STATE_PIN == 8, "Optional STATE input is D8");
static_assert(BT_STATE_PIN < 0 || !(MOTOR_PINS & (1UL << 8)), "STATE pin conflict");

#include "motor.h"
SoftwareSerial BTSerial(BT_RX, BT_TX);
constexpr uint8_t USB_LOCKED = 1, USB_WATCHDOG_EVENT = 2, BAD_FRAME_EVENT = 4;
constexpr uint8_t BT_ACTIVE = 8, HANDOFF_EVENT = 16, ESTOP_EVENT = 32;
constexpr uint32_t USB_TIMEOUT_MS = 400, PARTIAL_TIMEOUT_MS = 150;
bool usbLocked = true, btActive = false, btLocked = false, handoffPending = false;
uint8_t events = 0;
int usbLeft = 0, usbRight = 0, btPwm = 140;
int8_t btLeftDirection = 0, btRightDirection = 0;
uint32_t lastUsbAt = 0, lastBtAt = 0, lastSequence = 0;
bool haveSequence = false;
char usbLine[96];
uint8_t usbLength = 0;
bool lineActive = false, dropping = false;
uint32_t lineStarted = 0;

void resetParser() {
  usbLength = 0;
  lineActive = dropping = false;
}

void clearUsbInput() {
  resetParser();
  // Bound the work; stock ATmega328P HardwareSerial RX has a 64-byte ring.
  for (uint8_t i = 0; i < 64 && Serial.available(); ++i) Serial.read();
}

void stopUsb() {
  usbLocked = true;
  usbLeft = usbRight = 0;
  if (!btActive) setElectrical(0, 0);
}

void stopBoth(bool lockBluetooth) {
  btActive = false;
  btLocked = lockBluetooth;
  btLeftDirection = btRightDirection = 0;
  stopUsb();
  clearUsbInput();
}

bool bluetoothLinkReady() {
#if BT_STATE_PIN >= 0
  return digitalRead(BT_STATE_PIN) == HIGH;
#else
  return true;
#endif
}

void takeBluetooth(int8_t left, int8_t right) {
  if (btLocked || !bluetoothLinkReady()) return;
  if (!btActive) clearUsbInput();
  usbLocked = true;
  usbLeft = usbRight = 0;
  btActive = handoffPending = true;
  btLeftDirection = left;
  btRightDirection = right;
  lastBtAt = millis();
  setElectrical(left * btPwm, right * btPwm);
}

void setBluetoothSpeed(int pwm) {
  btPwm = pwm;
  if (btActive) setElectrical(btLeftDirection * btPwm, btRightDirection * btPwm);
}

void releaseBluetooth() {
  handoffPending = true;
  stopBoth(false);  // S/D is global stop plus explicit BT-latch release.
}

#include "move_case.h"

uint16_t crc16(const char *data, size_t length) {
  uint16_t crc = 0xFFFF;
  while (length--) {
    crc ^= (uint16_t)(uint8_t)*data++ << 8;
    for (uint8_t i = 0; i < 8; ++i)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

void ack(uint32_t sequence, bool clearHandoff) {
  uint8_t flags = events | (usbLocked ? USB_LOCKED : 0) |
      (btActive ? BT_ACTIVE : 0) | (handoffPending ? HANDOFF_EVENT : 0);
  char body[48], packet[56];
  snprintf(body, sizeof(body), "A %lu %d %d %u", (unsigned long)sequence,
      btActive ? motorLeft : usbLeft, btActive ? motorRight : usbRight, flags);
  int length = snprintf(packet, sizeof(packet), "%s*%04X\n", body, (unsigned)crc16(body, strlen(body)));
  // Do not let a USB flood block BT servicing. A dropped ACK is retried/recovered
  // by the host; event flags survive until an ACK actually enters the TX buffer.
  if (length <= 0 || length >= (int)sizeof(packet) || Serial.availableForWrite() < length) return;
  Serial.write((const uint8_t *)packet, length);
  events = 0;
  if (clearHandoff) handoffPending = false;
}

void badUsbFrame() {
  events |= BAD_FRAME_EVENT;
  if (!btActive) stopUsb();  // Corrupt USB must NEVER interrupt BT ownership.
}

bool unsignedField(const char *&text, uint32_t &value) {
  if (*text < '0' || *text > '9') return false;
  value = 0;
  while (*text >= '0' && *text <= '9') {
    uint8_t digit = *text++ - '0';
    if (value > (UINT32_MAX - digit) / 10) return false;
    value = value * 10 + digit;
  }
  return true;
}

bool pwmField(const char *&text, int &value) {
  bool negative = *text == '-';
  if (negative) ++text;
  uint32_t magnitude;
  if (!unsignedField(text, magnitude) || magnitude > 140) return false;
  value = negative ? -(int)magnitude : (int)magnitude;
  return true;
}

int hexDigit(char value) {
  if (value >= '0' && value <= '9') return value - '0';
  if (value >= 'A' && value <= 'F') return value - 'A' + 10;
  if (value >= 'a' && value <= 'f') return value - 'a' + 10;
  return -1;
}

void processUsbLine() {
  usbLine[usbLength] = '\0';
  char *star = strchr(usbLine, '*');
  if (!star || strlen(star + 1) != 4) { badUsbFrame(); return; }
  uint16_t checksum = 0;
  for (uint8_t i = 1; i <= 4; ++i) {
    int digit = hexDigit(star[i]);
    if (digit < 0) { badUsbFrame(); return; }
    checksum = (uint16_t)((checksum << 4) | digit);
  }
  if (crc16(usbLine, star - usbLine) != checksum) { badUsbFrame(); return; }
  *star = '\0';
  const char *text = usbLine;
  char kind = *text++;
  uint32_t sequence;
  if ((kind != 'M' && kind != 'E') || *text++ != ' ' || !unsignedField(text, sequence)) {
    badUsbFrame(); return;
  }
  if (kind == 'E') {
    if (*text != '\0') { badUsbFrame(); return; }
    if (btActive) handoffPending = true;
    stopBoth(true);
    events |= ESTOP_EVENT;
    ack(sequence, false);
    return;
  }
  int left, right;
  if (*text++ != ' ' || !pwmField(text, left) || *text++ != ' ' ||
      !pwmField(text, right) || *text != '\0') { badUsbFrame(); return; }
  if (btActive) { ack(sequence, false); return; }  // M0 polls; it never stops BT.
  if (left == 0 && right == 0) {
    // A zero handshake is also the sequence resynchronization point after a
    // host restart. No nonzero command can cross boot/watchdog/BT/E lock alone.
    usbLocked = false;
    haveSequence = true;
    lastSequence = sequence;
  } else {
    uint32_t delta = sequence - lastSequence;
    if (usbLocked) { ack(sequence, false); return; }
    if (!haveSequence || delta == 0 || delta >= 0x80000000UL) {
      badUsbFrame(); ack(sequence, false); return;
    }
    lastSequence = sequence;
  }
  lastUsbAt = millis();
  usbLeft = left;
  usbRight = right;
  setElectrical(left * ROS_LEFT_SIGN, right * ROS_RIGHT_SIGN);
  ack(sequence, left == 0 && right == 0);
}

void receiveUsbByte(char value) {
  if (value == '\n') {
    if (!dropping && usbLength) {
      if (usbLine[usbLength - 1] == '\r') --usbLength;
      if (usbLength) processUsbLine();
    }
    resetParser();
    return;
  }
  if (!lineActive) { lineActive = true; lineStarted = millis(); }
  if (dropping) return;
  if (value == '\0' || usbLength >= sizeof(usbLine) - 1) {
    badUsbFrame(); dropping = true; usbLength = 0; return;
  }
  usbLine[usbLength++] = value;
}

void checkSafety() {
  uint32_t now = millis();
  if (lineActive && (uint32_t)(now - lineStarted) >= PARTIAL_TIMEOUT_MS) {
    badUsbFrame(); resetParser();
  }
  bool btExpired = false;
#if BT_COMMAND_TIMEOUT_MS > 0
  btExpired = (uint32_t)(now - lastBtAt) >= BT_COMMAND_TIMEOUT_MS;
#endif
  if (btActive && (!bluetoothLinkReady() || btExpired)) {
    handoffPending = true;
    stopBoth(true);
  }
  if (!btActive && !usbLocked && (uint32_t)(now - lastUsbAt) >= USB_TIMEOUT_MS) {
    stopUsb();
    events |= USB_WATCHDOG_EVENT;
  }
}

void setup() {
  setupMotorPins();
  // Original sketch drove A0 HIGH. Its hardware purpose is unknown; preserve
  // it explicitly. It is NOT reassigned as e-stop or Bluetooth STATE.
  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
#if BT_STATE_PIN >= 0
  pinMode(BT_STATE_PIN, INPUT);
#endif
  Serial.begin(9600);
  BTSerial.begin(9600);
}

void loop() {
  checkSafety();
  for (uint8_t i = 0; i < 16 && BTSerial.available(); ++i) {
    move_case((char)BTSerial.read());
    checkSafety();
  }
  for (uint8_t i = 0; i < 32 && Serial.available(); ++i) {
    if (BTSerial.available()) move_case((char)BTSerial.read());
    checkSafety();
    if (Serial.available()) receiveUsbByte((char)Serial.read());
  }
  checkSafety();
}
