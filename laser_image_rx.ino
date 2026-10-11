/*
  Laser image receiver  (ESP32-S3 + SH1106 128x64 OLED)

  Library: U8g2 (Arduino Library Manager)

  OLED  -> ESP32-S3
    SCK      -> GPIO 12
    MOSI/SDA -> GPIO 11
    CS       -> GPIO 10
    DC       -> GPIO 47
    RST/RES  -> GPIO 21
    VCC/GND  -> 3.3V / GND
  Photodiode (raw analog) -> GPIO 5   (0-3.3V only)

  Start the transmitter first (or at the same time). At boot the receiver calibrates the
  threshold automatically from the transmitter's signal (midpoint between dark and light).
  Send 'c' in the Serial Monitor to recalibrate at any time.
*/

#include <Arduino.h>
#include <SPI.h>
#include <U8g2lib.h>

// ---------- Pins ----------
#define OLED_CS    5
#define OLED_DC    17
#define OLED_RST   16
const int SENSOR_PIN = 34;

// Hardware SPI uses the ESP32-S3 default FSPI pins: SCK=12, MOSI=11, CS=10
U8G2_SH1106_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);

// ---------- Protocol settings (must match the transmitter) ----------
const uint32_t BIT_US    = 1000;
const int      PAYLOAD   = 64;
const int      IMG_BYTES = 1024;
const int      PKT_COUNT = IMG_BYTES / PAYLOAD;   // 16

const int      MIN_SWING = 60;      // min ADC difference between laser ON and OFF (your signal: ~150)
const uint32_t RECAL_AFTER_MS = 30000;  // recalibrate if no valid packet for this long

// ---------- State ----------
int      threshold = 120;
uint8_t  frame[IMG_BYTES];
uint16_t gotMask = 0;
uint8_t  curSeq = 0;
bool     haveSeq = false;
bool     complete = false;
uint32_t crcErrors = 0, goodPackets = 0;
uint32_t lastGoodMs = 0;

// ---------- Sensor ----------
inline bool light() { return analogRead(SENSOR_PIN) > threshold; }

static inline void waitUntil(uint32_t t) {
  while ((int32_t)(micros() - t) < 0) { }
}

bool calibrate(uint32_t ms) {
  int mn = 4095, mx = 0;
  uint32_t t = millis();
  while (millis() - t < ms) {
    int v = 0;
    for (int i = 0; i < 4; i++) v += analogRead(SENSOR_PIN);
    v /= 4;
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  Serial.printf("Calibration: min %d, max %d\n", mn, mx);
  if (mx - mn < MIN_SWING) {
    Serial.println("No signal swing found (is the transmitter running and aligned?)");
    return false;
  }
  threshold = (mn + mx) / 2;
  Serial.printf("Threshold set to %d\n", threshold);
  return true;
}

// ---------- Software UART receive ----------
// Returns byte 0..255, or -1 timeout, -2 framing error, -3 false start.
int readByte(uint32_t timeoutUs) {
  uint32_t tw = micros();
  while (!light()) {                                   // wait for start bit (laser ON)
    if ((uint32_t)(micros() - tw) > timeoutUs) return -1;
  }
  uint32_t t0 = micros();
  waitUntil(t0 + BIT_US / 2);
  if (!light()) return -3;                             // glitch, not a real start bit

  uint8_t v = 0;
  for (int i = 0; i < 8; i++) {
    waitUntil(t0 + (uint32_t)(i + 1) * BIT_US + BIT_US / 2);   // middle of each data bit
    if (light()) v |= (1 << i);
  }
  waitUntil(t0 + 9UL * BIT_US + BIT_US / 2);           // middle of stop bit
  if (light()) return -2;                              // stop bit must be dark
  return v;
}

uint16_t crc16Update(uint16_t crc, uint8_t d) {       // CRC-16/CCITT-FALSE
  crc ^= (uint16_t)d << 8;
  for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

// ---------- Display ----------
void showMsg(const char *l1, const char *l2) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 14, l1);
  if (l2) u8g2.drawStr(0, 30, l2);
  u8g2.sendBuffer();
}

void showProgress(int got) {
  char line[28];
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tf);
  snprintf(line, sizeof(line), "Receiving image #%u", curSeq);
  u8g2.drawStr(0, 12, line);
  snprintf(line, sizeof(line), "Packets %d/%d", got, PKT_COUNT);
  u8g2.drawStr(0, 26, line);
  u8g2.drawFrame(0, 40, 128, 12);
  u8g2.drawBox(2, 42, (124 * got) / PKT_COUNT, 8);
  u8g2.sendBuffer();
}

void showImage() {
  u8g2.clearBuffer();
  u8g2.drawXBM(0, 0, 128, 64, frame);
  u8g2.sendBuffer();
}

// ---------- Packet handling ----------
// Called right after the 0xAA 0x55 sync bytes were seen.
void receivePacket() {
  uint8_t buf[2 + PAYLOAD + 2];
  for (size_t i = 0; i < sizeof(buf); i++) {
    int b = readByte(20000);
    if (b < 0) return;                                 // lost sync, go back to searching
    buf[i] = (uint8_t)b;
  }

  uint16_t crc = 0xFFFF;
  for (int i = 0; i < 2 + PAYLOAD; i++) crc = crc16Update(crc, buf[i]);
  uint16_t rxCrc = ((uint16_t)buf[2 + PAYLOAD] << 8) | buf[3 + PAYLOAD];
  if (crc != rxCrc) { crcErrors++; Serial.printf("CRC error (total %lu)\n", (unsigned long)crcErrors); return; }

  uint8_t seq = buf[0], idx = buf[1];
  if (idx >= PKT_COUNT) return;

  goodPackets++;
  lastGoodMs = millis();

  if (!haveSeq || seq != curSeq) {                     // new image starts
    curSeq = seq; haveSeq = true; gotMask = 0; complete = false;
  }
  if (complete) return;                                // repeat of an image we already have

  memcpy(frame + idx * PAYLOAD, buf + 2, PAYLOAD);
  gotMask |= (1u << idx);
  int got = __builtin_popcount(gotMask);
  Serial.printf("Image #%u packet %u OK (%d/%d)\n", seq, idx, got, PKT_COUNT);

  if (got == PKT_COUNT) {
    complete = true;
    showImage();
    Serial.printf("Image #%u complete. CRC errors so far: %lu\n", seq, (unsigned long)crcErrors);
  } else {
    showProgress(got);
  }
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  u8g2.begin();
  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);

  showMsg("Laser RX", "Calibrating...");
  while (!calibrate(2500)) showMsg("No laser signal", "Start transmitter");
  showMsg("Waiting for image", "...");
  lastGoodMs = millis();
}

void loop() {
  static uint8_t prev = 0;

  if (Serial.available() && Serial.read() == 'c') {
    showMsg("Recalibrating...", nullptr);
    calibrate(2500);
    lastGoodMs = millis();
  }

  if (millis() - lastGoodMs > RECAL_AFTER_MS) {        // lost the signal for a while
    lastGoodMs = millis();
    calibrate(2500);
  }

  int b = readByte(50000);                             // 50 ms timeout keeps loop() responsive
  if (b < 0) { prev = 0; return; }

  if (prev == 0xAA && b == 0x55) { receivePacket(); prev = 0; }
  else prev = (uint8_t)b;
}
