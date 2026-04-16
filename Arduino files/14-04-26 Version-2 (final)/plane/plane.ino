#include <Arduino.h>
#include <Wire.h>
#include <RF24.h>
#include <SPI.h>
#include <FastLED.h>
#include <Servo.h>

// ---------------- IMU ----------------
#define IMU_SDA 4
#define IMU_SCL 5
#define MPU_ADDR 0x68

// ---------------- NRF ----------------
#define NRF_RX 0
#define NRF_CS 1
#define NRF_SCK 2
#define NRF_TX 3
#define NRF_CE 6

// ---------------- OUTPUT ----------------
#define PORT_SERVO 27
#define STAR_SERVO 26
#define EL_SERVO   15
#define PROP       28

// ---------------- LED ----------------
#define LED_PIN 16
const int BRIGHTNESS = 120;
CRGB led[1];

// ---------------- DATA ----------------
// rtt_timer is being used as a packet ID / echo tag.
struct __attribute__((packed)) Packet_TX {
  uint8_t gyro_x;
  uint8_t gyro_y;
  uint8_t gyro_z;
  uint8_t rtt_timer;
};

struct __attribute__((packed)) Packet_RX {
  uint16_t servo_write[4];
  uint8_t rtt_timer;
};

static_assert(sizeof(Packet_TX) == 4, "Packet_TX size wrong");
static_assert(sizeof(Packet_RX) == 9, "Packet_RX size wrong");

// ---------------- GLOBALS ----------------
Packet_TX GLOBAL_TX = {128, 128, 128, 0};
Packet_RX GLOBAL_RX = {{1500, 1500, 1500, 1000}, 0};

uint32_t lastLinkTime = 0;
bool linkOK = false;
uint32_t rxCount = 0;
uint32_t txCount = 0;

// ---------------- RADIO ----------------
RF24 radio(NRF_CE, NRF_CS);

byte rxAddr[6] = "CNTRL";
byte txAddr[6] = "PLANE";

// ---------------- SERVO OBJECTS ----------------
Servo servo_port;
Servo servo_star;
Servo servo_el;
Servo esc;

// ---------------- PROTOTYPES ----------------
void input();
bool radioUpdate();
void updateMotor();
void updateLED();
void debugOutput();

// ======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Plane.");

  FastLED.addLeds<WS2812, LED_PIN, GRB>(led, 1);
  led[0] = CRGB(255 - BRIGHTNESS, 255 - BRIGHTNESS, 255 - BRIGHTNESS);
  FastLED.show();

  // Servo / ESC setup
  servo_port.attach(PORT_SERVO);
  servo_star.attach(STAR_SERVO);
  servo_el.attach(EL_SERVO);
  esc.attach(PROP);

  // Safe startup values
  esc.writeMicroseconds(1000);
  servo_port.writeMicroseconds(1500);
  servo_star.writeMicroseconds(1500);
  servo_el.writeMicroseconds(1500);

  delay(2000);  // ESC arm time

  // SPI / Radio
  SPI.setRX(NRF_RX);
  SPI.setCS(NRF_CS);
  SPI.setSCK(NRF_SCK);
  SPI.setTX(NRF_TX);
  SPI.begin();

  if (!radio.begin()) {
    Serial.println("RADIO NOT FOUND");
  } else {
    Serial.println("RADIO FOUND");
  }

  // Same settings that matched your working test
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.setChannel(108);

  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1, rxAddr);
  radio.startListening();
  radio.flush_rx();
  radio.flush_tx();

  // IMU
  Wire.setSDA(IMU_SDA);
  Wire.setSCL(IMU_SCL);
  Wire.begin();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

// ======================================================
void loop() {
  // Always poll radio as fast as possible
  radioUpdate();

  static uint32_t lastTick = 0;
  uint32_t now = millis();

  if (now - lastTick >= 10) {   // 100 Hz
    lastTick += 10;

    input();
    updateMotor();
    updateLED();
    debugOutput();
  }
}

// ======================================================
/*
void input() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) {
    return;
  }

  Wire.read(); Wire.read(); // ax
  Wire.read(); Wire.read(); // ay
  Wire.read(); Wire.read(); // az
  Wire.read(); Wire.read(); // temp

  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // compress gyro to 0..255
  GLOBAL_TX.gyro_x = (uint8_t)((gx >> 8) + 128);
  GLOBAL_TX.gyro_y = (uint8_t)((gy >> 8) + 128);
  GLOBAL_TX.gyro_z = (uint8_t)((gz >> 8) + 128);
}
*/
void input() {
  static uint8_t n = 0;
  n++;

  GLOBAL_TX.gyro_x = n;
  GLOBAL_TX.gyro_y = n + 1;
  GLOBAL_TX.gyro_z = n + 2;
}

// ======================================================
bool radioUpdate() {
  bool gotPacket = false;

  // Drain RX queue, keep newest control packet
  while (radio.available()) {
    radio.read(&GLOBAL_RX, sizeof(GLOBAL_RX));
    rxCount++;
    gotPacket = true;
  }

  if (gotPacket) {
    lastLinkTime = millis();
    linkOK = true;

    // Echo back the packet ID from controller
    GLOBAL_TX.rtt_timer = GLOBAL_RX.rtt_timer;

    radio.stopListening();
    bool ok = radio.write(&GLOBAL_TX, sizeof(GLOBAL_TX));
    radio.startListening();

    if (ok) {
      txCount++;
    }

    return true;
  }

  if (millis() - lastLinkTime > 100) {
    linkOK = false;
  }

  return false;
}

// ======================================================
void updateMotor() {
  static uint16_t current[4] = {1500, 1500, 1500, 1000};

  for (int i = 0; i < 4; i++) {
    uint16_t target;

    if (!linkOK) {
      target = (i == 3) ? 1000 : 1500;
    } else {
      // assuming controller already clamps values properly
      target = GLOBAL_RX.servo_write[i];
    }

    // faster smoothing
    if (i == 3) {
      current[i] = (current[i] + target) / 2;       // throttle: 2-tick
    } else {
      current[i] = (current[i] * 2 + target) / 3;   // surfaces: 3-tick
    }
  }

  servo_port.writeMicroseconds(current[0]);
  servo_star.writeMicroseconds(current[1]);
  servo_el.writeMicroseconds(current[2]);
  esc.writeMicroseconds(current[3]);
}

// ======================================================
void updateLED() {
  if (!linkOK) {
    led[0] = CRGB(BRIGHTNESS, 0, 0);   // red = no valid control packets
  } else {
    uint8_t level = map(GLOBAL_RX.servo_write[3], 1000, 2000, 0, BRIGHTNESS);
    led[0] = CRGB(0, level, 0);        // green brightness = throttle
  }

  FastLED.show();
}

// ======================================================
void debugOutput() {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();

  if (now - lastPrint < 100) return;   // 10 Hz serial output
  lastPrint = now;

  Serial.print("LINK: ");
  Serial.print(linkOK ? "OK" : "LOST");

  Serial.print(" | RX#: ");
  Serial.print(rxCount);

  Serial.print(" | TX#: ");
  Serial.print(txCount);

  Serial.print(" | IN: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(GLOBAL_RX.servo_write[i]);
    if (i < 3) Serial.print(", ");
  }

  Serial.print(" | GYRO: ");
  Serial.print(GLOBAL_TX.gyro_x);
  Serial.print(", ");
  Serial.print(GLOBAL_TX.gyro_y);
  Serial.print(", ");
  Serial.print(GLOBAL_TX.gyro_z);

  Serial.print(" | ECHO_ID: ");
  Serial.print(GLOBAL_TX.rtt_timer);

  Serial.println();
}