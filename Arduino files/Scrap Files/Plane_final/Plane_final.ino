#include <Wire.h>
#include <RF24.h>
#include <SPI.h>
#include <FastLED.h>
//#include <Servo.h>

#define IMU_SDA 4
#define IMU_SCL 5

#define NRF_RX 0
#define NRF_CS 1
#define NRF_SCK 2
#define NRF_TX 3
#define NRF_CE 6

#define PORT_SERVO 27
#define STAR_SERVO 26
#define EL_SERVO 15
#define PROP 28

#define MPU_ADDR 0x68

struct Packet_TX{
  union {
    struct {
      uint8_t gyro_x;
      uint8_t gyro_y;
      uint8_t gyro_z;
      uint8_t rtt_timer;
    };
    uint8_t access[4];
  };
};

struct Packet_RX{
  union {
    struct {
      uint16_t servo_write[4];
      uint8_t rtt_timer;
      uint8_t padding;   // force alignment
    };
    uint8_t access[10];
  };
};

const uint8_t MOTORS[4] = {PORT_SERVO, STAR_SERVO, EL_SERVO, PROP};

void input(Packet_TX &p);
void updateMotor(Packet_RX &p);
void radioHandler(Packet_TX &send, Packet_RX &receive);
void debugOutput(const Packet_RX &p);

RF24 radio(NRF_CE, NRF_CS);

byte rxAddr[6] = "CNTRL";
byte txAddr[6] = "PLANE";

uint32_t lastLinkTime=0;
bool linkOK = false;

Packet_TX GLOBAL_TX;
Packet_RX GLOBAL_RX;

void setup() {
  Serial.begin(115200);
  analogWriteFreq(50);
  analogWriteRange(20000);

  for (int i = 0; i < 4; i++) {
    pinMode(MOTORS[i], OUTPUT);
  }
  SPI.setRX(NRF_RX);
  SPI.setCS(NRF_CS);
  SPI.setSCK(NRF_SCK);
  SPI.setTX(NRF_TX);
  SPI.begin();
  
  Wire.setSDA(IMU_SDA);
  Wire.setSCL(IMU_SCL);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

    if (!radio.begin()) {
    Serial.println("RADIO NOT FOUND");
  } else {
    Serial.println("RADIO FOUND");
  }

  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(0,0);
  radio.printDetails();
  Serial.println(radio.isChipConnected());
  radio.setChannel(108);

  radio.enableAckPayload();

  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1, rxAddr);

  radio.startListening();


}

void loop() {
  static uint32_t lastRadio = 0;
  static uint32_t lastDisplay = 0;

  uint32_t now = millis();

  // --- 100Hz radio update ---
  if (now - lastRadio >= 10) {
    radioHandler(GLOBAL_TX, GLOBAL_RX);
    lastRadio += 10;
  }
}

void input(Packet_TX &p) {
  static int32_t lastAccel=0;
  static int32_t lastIMUchange=0;
  static bool imuWorking=false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);            // start at ACCEL_XOUT_H
  Wire.endTransmission(false);

  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) return;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();

  Wire.read(); 
  Wire.read();    // skip temperature

  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // compress into 0–255


  p.gyro_x = (gx >> 8) + 128;
  p.gyro_y = (gy >> 8) + 128;
  p.gyro_z = (gz >> 8) + 128;
  
  if (ax != lastAccel) {
    imuWorking = true;
    lastIMUchange = millis();
  }

  lastAccel = ax;

  // detect frozen IMU
  if (millis() - lastIMUchange > 500) {
    imuWorking = false;
  }
}

void updateMotor(Packet_RX &p) {
  if (!linkOK) {
    analogWrite(PROP, 1000); // motor off
    analogWrite(PORT_SERVO, 1500);
    analogWrite(STAR_SERVO, 1500);
    analogWrite(EL_SERVO, 1500);
  return;
  }

  for (int k=0; k<4; k++) {
    analogWrite(MOTORS[k], p.servo_write[k]);
  }
}

void radioHandler(Packet_TX &send, Packet_RX &receive) {
  if (radio.available()) {
    radio.read(receive.access, sizeof(receive.access));
    send.rtt_timer = receive.rtt_timer;
    debugOutput(receive);
  }
  updateMotor(receive);

  input(send);
  radio.stopListening();
  bool success = radio.write(send.access, sizeof(send.access));
  radio.startListening();

  if (success) {
    lastLinkTime = millis();
    linkOK = true;
  }

  if (millis() - lastLinkTime > 100) {   // 100ms timeout
    linkOK = false;
  }
}

void debugOutput(const Packet_RX &p) {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();

  // limit spam (10 Hz output)
  if (now - lastPrint < 100) return;
  lastPrint = now;

  Serial.print("LINK: ");
  Serial.print(linkOK ? "OK" : "LOST");

  Serial.print(" | SERVO: ");

  for (int i = 0; i < 4; i++) {
    Serial.print(p.servo_write[i]);
    if (i < 3) Serial.print(", ");
  }

  Serial.print(" | RTT: ");
  Serial.print(p.rtt_timer);

  Serial.println();
}