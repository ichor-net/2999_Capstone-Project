#include <Wire.h>
#include <RF24.h>
#include <SPI.h>
#include <FastLED.h>
#include <Servo.h>

#define IMU_SDA 4
#define IMU_SCL 5

#define NRF_RX 0
#define NRF_CS 1
#define NRF_SCK 2
#define NRF_TX 3
#define NRF_CE 6

#define MOTOR_ESC 1
#define LEFT_AILERON 1
#define RIGHT_AILERON 1
#define ELEVATOR_1 1

#define MPU_ADDR 0x68

struct Packet_RX{
  union {
    struct {
      uint16_t x_byte;
      uint16_t y_byte;
      uint16_t power;
      uint16_t rtt_timer;
    };
    uint16_t access[4];
  };
};

struct Packet_TX{
  union {
    struct {
        uint8_t gyro_x;
        uint8_t gyro_y;
        uint8_t gyro_z;
        uint8_t accel_x;
        uint8_t accel_y;
        uint8_t accel_z;
        uint8_t rtt_timer;
    };
      uint8_t access[7];
  };
};

struct Control{
  uint32_t lastLinkTime = 0;
  bool linkOK = false;
  uint8_t lastAccel = 0;
  bool imuWorking = false;
  uint32_t lastIMUchange = 0;
};

Packet_TX tx;
Packet_RX rx;
Control c_mod;

byte rxAddr[6] = "PLANE";
byte txAddr[6] = "CNTRL";
const int BRIGHTNESS = 128;
CRGB led[1];

RF24 radio(NRF_CE, NRF_CS);

Servo servos[3];

void setup() {
  analogWriteFreq(50);
  analogWriteRange(20000);
  FastLED.addLeds<WS2812, 16, GRB>(led, 1);
  led[0] = CRGB(255-BRIGHTNESS, 255-BRIGHTNESS, 255-BRIGHTNESS);
  FastLED.show();
  delay(1000);

  led[0] = CRGB(255-BRIGHTNESS, 255-BRIGHTNESS, 0);
  FastLED.show();

  Wire.setSDA(IMU_SDA);
  Wire.setSCL(IMU_SCL);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(1000);

  if (!radio.begin()){ 
    led[0] = CRGB(255-BRIGHTNESS, 0, 0);
    FastLED.show();
    while(!radio.begin());
  }
  led[0] = CRGB(255-BRIGHTNESS, 0, 255-BRIGHTNESS);
  FastLED.show();
  
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_2MBPS);
  radio.setRetries(0,0);
  radio.setChannel(108);
  radio.enableAckPayload();
  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1, rxAddr);
  radio.startListening();
  delay(1000);

  led[0] = CRGB(0, 255-BRIGHTNESS, 0);
  FastLED.show();
}
const uint32_t LOOP_HZ = 250.0f;
const uint32_t LOOP_PERIOD_US = 1000000.0f / LOOP_HZ; 

void loop() {
  static uint32_t lastTime = micros();
  uint32_t now = micros();
  static bool link = !c_mod.linkOK;

  while (now - lastTime >= LOOP_PERIOD_US) {
    lastTime += LOOP_PERIOD_US;
    radioHandler(tx, rx, c_mod);

    if (link!=c_mod.linkOK) {
      link=c_mod.linkOK;
      if (!link) {
        led[0] = CRGB(255-BRIGHTNESS, 0, 0);
      }
      else {
        led[0] = CRGB(0, 255-BRIGHTNESS, 0);
      }
      FastLED.show();
    }
  }
}

void radioHandler(Packet_TX &send, Packet_RX &receive, Control &c_local) {

  if (radio.available()) {
    radio.read(receive.access, sizeof(receive.access));
    send.rtt_timer = receive.rtt_timer;
  }

  dataLoader(send, c_local);
  radio.stopListening();
  bool success = radio.write(send.access, sizeof(send.access));
  radio.startListening();

  if (success) {
    c_local.lastLinkTime = millis();
    c_local.linkOK = true;
  }

  if (millis() - c_local.lastLinkTime > 100) {   // 100ms timeout
    c_local.linkOK = false;
  }
}

void dataLoader(Packet_TX &p, Control &c) {
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);            // start at ACCEL_XOUT_H
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  Wire.read(); 
  Wire.read();    // skip temperature

  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // compress into 0–255
  p.accel_x = map(ax, -32768, 32767, 0, 255);
  p.accel_y = map(ay, -32768, 32767, 0, 255);
  p.accel_z = map(az, -32768, 32767, 0, 255);

  p.gyro_x = map(gx, -32768, 32767, 0, 255);
  p.gyro_y = map(gy, -32768, 32767, 0, 255);
  p.gyro_z = map(gz, -32768, 32767, 0, 255);
  
  if (p.accel_x != c.lastAccel) {
    c.imuWorking = true;
    c.lastIMUchange = millis();
  }

  c.lastAccel = p.accel_x;

  // detep frozen IMU
  if (millis() - c.lastIMUchange > 500) {
    c.imuWorking = false;
  }
}

void updateMotor(Packet_RX &p) {
  for (int k=0; k<3; k++) {
    analogWrite(p.access[k]+1000, MOTORS[k]);
  }
}


