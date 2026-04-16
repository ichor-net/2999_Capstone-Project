#include <Arduino.h>
#include <FastLED.h>
#include <Servo.h>

// ---------------- OUTPUT ----------------
#define PORT_SERVO 5
#define STAR_SERVO 6
#define EL_SERVO   7
#define PROP       8

// ---------------- LED ----------------
#define LED_PIN 16
const int BRIGHTNESS = 200;
CRGB led[1];

// ---------------- DATA ----------------
struct Packet {
  uint16_t servo[4];
  uint8_t rtt;
};

Packet rx;

uint32_t lastLink = 0;
bool linkOK = false;

// smoothing
uint16_t currentServo[4] = {1500,1500,1500,1000};

// ---------------- SERVO OBJECTS ----------------
Servo servo_port;
Servo servo_star;
Servo servo_el;
Servo esc;

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

  // attach servos
  servo_port.attach(PORT_SERVO);
  servo_star.attach(STAR_SERVO);
  servo_el.attach(EL_SERVO);
  esc.attach(PROP);

  // LED
  FastLED.addLeds<WS2812, LED_PIN, GRB>(led, 1);
  led[0] = CRGB(255 - BRIGHTNESS, 255 - BRIGHTNESS, 255 - BRIGHTNESS);
  FastLED.show();

  // safe startup
  esc.writeMicroseconds(1000);
  servo_port.writeMicroseconds(1500);
  servo_star.writeMicroseconds(1500);
  servo_el.writeMicroseconds(1500);

  delay(2000); // ESC arm time
}

// ======================================================
void loop() {
  readPacket();
  updateMotors();
  updateLED();
}

// ======================================================
void readPacket() {
  static uint8_t buf[11];
  static int index = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    if (index == 0 && b != 0xAA) continue;

    buf[index++] = b;

    if (index == 11) {
      uint8_t check = checksum(&buf[1], 9);

      if (check == buf[10]) {
        int i = 1;

        for (int k = 0; k < 4; k++) {
          rx.servo[k] = (buf[i] << 8) | buf[i + 1];
          i += 2;
        }

        rx.rtt = buf[i];

        lastLink = millis();
        linkOK = true;
      }

      index = 0;
    }
  }

  if (millis() - lastLink > 100) {
    linkOK = false;
  }
}

// ======================================================
void updateMotors() {
  for (int i = 0; i < 4; i++) {
    uint16_t target;

    if (!linkOK) {
      target = (i == 3) ? 1000 : 1500;
    } else {
      target = rx.servo[i];
    }

    // smoothing
    currentServo[i] = (currentServo[i] * 3 + target) / 4;
  }

  servo_port.writeMicroseconds(currentServo[0]);
  servo_star.writeMicroseconds(currentServo[1]);
  servo_el.writeMicroseconds(currentServo[2]);
  esc.writeMicroseconds(currentServo[3]);
}

// ======================================================
void updateLED() {
  if (!linkOK) {
    led[0] = CRGB(BRIGHTNESS, 0, 0); // red = lost link
  } else {
    uint8_t level = map(currentServo[3], 1000, 2000, 0, BRIGHTNESS);
    led[0] = CRGB(0, level, 0);
  }

  FastLED.show();
}