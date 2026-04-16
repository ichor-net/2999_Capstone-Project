#include <Arduino.h>
#include <FastLED.h>

// ---------------- INPUT ----------------
#define JSTK_X   29
#define JSTK_Y   28
#define SLD_POT  26

// ---------------- LED ----------------
#define LED_PIN 16
const int BRIGHTNESS = 200;
CRGB led[1];

// ---------------- DATA ----------------
struct Packet {
  uint16_t servo[4];
  uint8_t rtt;
};

Packet tx;

// smoothing
int sx = 512, sy = 512, st = 0;

// ======================================================
uint8_t checksum(uint8_t *data, int len) {
  uint8_t c = 0;
  for (int i = 0; i < len; i++) c ^= data[i];
  return c;
}

// ======================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(led, 1);
  led[0] = CRGB(255 - BRIGHTNESS, 255 - BRIGHTNESS, 255 - BRIGHTNESS);
  FastLED.show();
}

// ======================================================
void loop() {
  static uint32_t last = 0;
  uint32_t now = millis();

  if (now - last >= 10) {
    readInputs();
    sendPacket();
    updateLED();
    last = now;
  }
}

// ======================================================
void readInputs() {
  const int MIN = 1000;
  const int MAX = 2000;

  sx = (sx * 3 + analogRead(JSTK_X)) / 4;
  sy = (sy * 3 + analogRead(JSTK_Y)) / 4;
  st = (st * 3 + analogRead(SLD_POT)) / 4;

  tx.rtt = millis() & 0xFF;

  tx.servo[0] = constrain(map(sx, 0, 1023, MIN, MAX), MIN, MAX);
  tx.servo[1] = constrain(map(sx, 0, 1023, MAX, MIN), MIN, MAX);
  tx.servo[2] = constrain(map(sy, 0, 1023, MIN, MAX), MIN, MAX);
  tx.servo[3] = constrain(map(st, 0, 1023, MIN, MAX), MIN, MAX);
}

// ======================================================
void sendPacket() {
  uint8_t buf[11];
  int i = 0;

  buf[i++] = 0xAA;

  for (int k = 0; k < 4; k++) {
    buf[i++] = tx.servo[k] >> 8;
    buf[i++] = tx.servo[k] & 0xFF;
  }

  buf[i++] = tx.rtt;
  buf[i] = checksum(&buf[1], 9);

  Serial1.write(buf, 11);
}

// ======================================================
void updateLED() {
  uint8_t level = map(tx.servo[3], 1000, 2000, 0, BRIGHTNESS);
  led[0] = CRGB(level, 0, 255 - level);
  FastLED.show();
}