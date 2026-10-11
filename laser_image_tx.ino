/*
  Laser image transmitter  (ESP32 DevKit)

  Sends 128x64 1-bit images (from images.h) over a laser using a software UART:
    idle = laser OFF, start bit = laser ON, 8 data bits (LSB first, 1 = ON), stop bit = OFF.
  The fast turn-ON edge is used for timing, so the slow turn-OFF of the laser/sensor doesn't hurt.

  Packet format (70 bytes):
    0xAA 0x55 | seq | index | 64 bytes payload | CRC16 hi | CRC16 lo
  One image = 1024 bytes = 16 packets. Each image is repeated REPEATS times so the receiver
  can fill in any packet that was corrupted (there is no return channel).

  Wiring:
    Laser module signal -> GPIO 25 (use a transistor/MOSFET if it draws > 20 mA)
    Laser GND           -> GND

  Make your own images.h with:  python make_images.py pic1.png pic2.png ...

  SAFETY: Never look into the laser beam.
*/

#include <Arduino.h>
#include "images.h"

// ---------------- Settings ----------------
const int      LASER_PIN      = 21;
const uint32_t BIT_US         = 1000;  // 1000 us = 1000 baud. Must match the receiver.
                                       // Keep BIT_US >= ~3x your turn-OFF time (measured ~200 us).
                                       // After it works, try 800 or 600 for more speed.
const int      PAYLOAD        = 64;    // bytes per packet
const int      PKT_COUNT      = IMG_BYTES / PAYLOAD;   // 16
const uint32_t PACKET_GAP_MS  = 60;    // laser OFF between packets (receiver draws to OLED here)
const int      REPEATS        = 2;     // times each image is sent
const uint32_t REPEAT_GAP_MS  = 1000;

uint8_t seqNo = 0;

// ---------------- Helpers ----------------
inline void setLaser(bool on) { digitalWrite(LASER_PIN, on ? HIGH : LOW); }

static inline void waitUntil(uint32_t t) {
  while ((int32_t)(micros() - t) < 0) { }
}

uint16_t crc16Update(uint16_t crc, uint8_t d) {       // CRC-16/CCITT-FALSE
  crc ^= (uint16_t)d << 8;
  for (int i = 0; i < 8; i++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  return crc;
}

void sendByte(uint8_t b) {
  uint32_t t0 = micros();
  setLaser(true);                                     // start bit
  waitUntil(t0 + BIT_US);
  for (int i = 0; i < 8; i++) {                       // data bits, LSB first
    setLaser((b >> i) & 1);
    waitUntil(t0 + (uint32_t)(i + 2) * BIT_US);
  }
  setLaser(false);                                    // stop bit
  waitUntil(t0 + 10UL * BIT_US);
}

void sendPacket(uint8_t seq, uint8_t idx, const uint8_t *payload) {
  uint16_t crc = 0xFFFF;
  crc = crc16Update(crc, seq);
  crc = crc16Update(crc, idx);
  for (int i = 0; i < PAYLOAD; i++) crc = crc16Update(crc, payload[i]);

  sendByte(0xAA);
  sendByte(0x55);
  sendByte(seq);
  sendByte(idx);
  for (int i = 0; i < PAYLOAD; i++) sendByte(payload[i]);
  sendByte(crc >> 8);
  sendByte(crc & 0xFF);
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  pinMode(LASER_PIN, OUTPUT);
  setLaser(false);

  float secPerPass = PKT_COUNT * ((PAYLOAD + 6) * 10.0f * BIT_US / 1e6f + PACKET_GAP_MS / 1000.0f);
  Serial.printf("Laser image TX: %d image(s), %lu baud, ~%.1f s per pass\n",
                NUM_IMAGES, (unsigned long)(1000000UL / BIT_US), secPerPass);
  Serial.println("Starting in 3 seconds (power the receiver and let it calibrate)...");
  delay(3000);
}

void loop() {
  for (int img = 0; img < NUM_IMAGES; img++) {
    seqNo++;                                          // new number = receiver starts a new image
    for (int r = 0; r < REPEATS; r++) {
      Serial.printf("Sending image %d/%d (pass %d/%d)\n", img + 1, NUM_IMAGES, r + 1, REPEATS);
      for (int p = 0; p < PKT_COUNT; p++) {
        sendPacket(seqNo, p, IMAGES[img] + p * PAYLOAD);
        setLaser(false);
        delay(PACKET_GAP_MS);
      }
      delay(REPEAT_GAP_MS);
    }
  }
}
