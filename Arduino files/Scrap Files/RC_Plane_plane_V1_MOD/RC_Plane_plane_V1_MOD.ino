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

void input();
void updateMotor();
bool radioUpdate();

struct Packet_RX{
  uint16_t servo_write[4];
  uint8_t rtt_timer;
};

struct Packet_TX{
  uint8_t gyro_x;
  uint8_t gyro_y;
  uint8_t gyro_z;
  uint8_t rtt_timer;
};

const uint8_t MOTORS[4] = {PORT_SERVO, STAR_SERVO, EL_SERVO, PROP};

uint32_t lastLinkTime = 0;
bool linkOK = false;
uint8_t lastAccel = 0;
bool imuWorking = false;
uint32_t lastIMUchange = 0;

byte txAddr[6] = "NODE2";
byte rxAddr[6] = "NODE1";

Packet_TX GLOBAL_TX;
Packet_RX GLOBAL_RX;

RF24 radio(NRF_CE, NRF_CS);

void setup() {
  Serial.begin(115200);
  delay(1500);


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
  radio.setDataRate(RF24_2MBPS);
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

  input();

  // --- 100Hz radio update ---
  if (now - lastRadio >= 10) {
    radioUpdate();
    updateMotor();
    lastRadio += 10;
  }

}

void input() {
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
  //GLOBAL_TX.accel_x = map(ax, -32768, 32767, 0, 255);
  //GLOBAL_TX.accel_y = map(ay, -32768, 32767, 0, 255);
  //GLOBAL_TX.accel_z = map(az, -32768, 32767, 0, 255);

  GLOBAL_TX.gyro_x = map(gx, -32768, 32767, 0, 255);
  GLOBAL_TX.gyro_y = map(gy, -32768, 32767, 0, 255);
  GLOBAL_TX.gyro_z = map(gz, -32768, 32767, 0, 255);
  
  if (ax != lastAccel) {
    imuWorking = true;
    lastIMUchange = millis();
  }

  lastAccel = ax;

  // deteGLOBAL_TX frozen IMU
  if (millis() - lastIMUchange > 500) {
    imuWorking = false;
  }
}

bool radioUpdate(){

  if (radio.available()) {

    radio.read(&GLOBAL_RX, sizeof(GLOBAL_RX));

    radio.stopListening();
    GLOBAL_TX.rtt_timer = GLOBAL_RX.rtt_timer;
    radio.write(&GLOBAL_TX, sizeof(GLOBAL_TX));
    radio.startListening();

    lastLinkTime = millis();
    linkOK = true;
    return true;
  }

  if (millis() - lastLinkTime > 100) {
    linkOK = false;
  }

  return false;
}

void updateMotor() {
  if (!linkOK) {
    analogWrite(PROP, 1000); // motor off
    analogWrite(PORT_SERVO, 1500);
    analogWrite(STAR_SERVO, 1500);
    analogWrite(EL_SERVO, 1500);
  return;
  }

  for (int k=0; k<4; k++) {
    analogWrite(MOTORS[k], GLOBAL_RX.servo_write[k]);
  }
}
