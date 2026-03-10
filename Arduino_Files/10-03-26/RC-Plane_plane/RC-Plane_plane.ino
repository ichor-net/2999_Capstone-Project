#include <U8g2lib.h>
#include <Wire.h>
#include <RF24.h>
#include <SPI.h>

#define IMU_SDA 6
#define IMU_SCL 7

#define OLED_SDA 4
#define OLED_SCL 5

#define RX 0
#define CS 1
#define SCK 2
#define TX 3
#define CE 8

#define MPU_ADDR 0x68

void input();
void draw();
bool radioUpdate();

struct Packet_RX{
  uint8_t x;
  uint8_t y;
  uint8_t t;
  uint8_t b;
  uint8_t time;
};

struct Packet_TX{
  uint8_t gyro_x;
  uint8_t gyro_y;
  uint8_t gyro_z;
  uint8_t accel_x;
  uint8_t accel_y;
  uint8_t accel_z;
  uint8_t time;
};

const char *names[] = {
  "GX:",
  "GY:",
  "GZ:",
  "AX:",
  "AY:",
  "AZ:"
};

uint32_t lastLinkTime = 0;
bool linkOK = false;
uint8_t lastAccel = 0;
bool imuWorking = false;
uint32_t lastIMUchange = 0;

byte txAddr[6] = "NODE2";
byte rxAddr[6] = "NODE1";

Packet_TX ct;
Packet_RX cr;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R0, U8X8_PIN_NONE);
RF24 radio(CE, CS);

void setup() {
  Serial.begin(115200);
  delay(1500);


  SPI.setRX(RX);
  SPI.setCS(CS);
  SPI.setSCK(SCK);
  SPI.setTX(TX);
  SPI.begin();

  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  Wire.begin();

  Wire1.setSDA(IMU_SDA);
  Wire1.setSCL(IMU_SCL);
  Wire1.begin();
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x6B);
  Wire1.write(0);
  Wire1.endTransmission(true);

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

  disp.begin();
  //disp.setFont(u8g2_font_5x7_tf);
  disp.setFont(u8g2_font_squeezed_r6_tr);
  disp.clearBuffer();
}

void loop() {

  static uint32_t lastRadio = 0;
  static uint32_t lastDisplay = 0;

  uint32_t now = millis();

  input();

  // --- 100Hz radio update ---
  if (now - lastRadio >= 10) {
    radioUpdate();
    lastRadio += 10;
  }

  // --- 60Hz display update ---
  if (now - lastDisplay >= 16) {
    draw();
    lastDisplay = now;
  }
}

void input() {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(0x3B);            // start at ACCEL_XOUT_H
  Wire1.endTransmission(false);

  Wire1.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = (Wire1.read() << 8) | Wire1.read();
  int16_t ay = (Wire1.read() << 8) | Wire1.read();
  int16_t az = (Wire1.read() << 8) | Wire1.read();

  Wire1.read(); 
  Wire1.read();    // skip temperature

  int16_t gx = (Wire1.read() << 8) | Wire1.read();
  int16_t gy = (Wire1.read() << 8) | Wire1.read();
  int16_t gz = (Wire1.read() << 8) | Wire1.read();

  // compress into 0–255
  ct.accel_x = map(ax, -32768, 32767, 0, 255);
  ct.accel_y = map(ay, -32768, 32767, 0, 255);
  ct.accel_z = map(az, -32768, 32767, 0, 255);

  ct.gyro_x = map(gx, -32768, 32767, 0, 255);
  ct.gyro_y = map(gy, -32768, 32767, 0, 255);
  ct.gyro_z = map(gz, -32768, 32767, 0, 255);
  
  if (ct.accel_x != lastAccel) {
    imuWorking = true;
    lastIMUchange = millis();
  }

  lastAccel = ct.accel_x;

  // detect frozen IMU
  if (millis() - lastIMUchange > 500) {
    imuWorking = false;
  }
}

void draw() {
  disp.clearBuffer();

  disp.drawVLine(51, 6, 50);
  disp.drawDisc(51, 56 - map(cr.t, 0, 255, 0, 50), 2);

  disp.drawCircle(93, 31, 30);

  int px = map(cr.x, 0, 255, 63, 123);
  int py = map(cr.y, 0, 255, 61, 1);

  int dx = px - 93;
  int dy = py - 31;

  if (dx * dx + dy * dy > 30 * 30) {
    float a = atan2(dy, dx);
    px = 93 + cos(a) * 30;
    py = 31 + sin(a) * 30;
  }
  disp.drawDisc(px, py, 2);

  if (cr.b==0) {
    disp.drawCircle(61, 6, 4);
  }
  else {
    disp.drawDisc(61, 6, 3);
  }

  uint8_t *data = (uint8_t*)&ct;
  for (uint8_t i = 0; i < sizeof(ct)-1; i++) {
    char buf[16];
    sprintf(buf, "%s %u", names[i], data[i]);

    disp.drawStr(0, 10 + i*10, buf);
  }

  if (linkOK) {
    disp.drawStr(30, 8, "LINK");
  }
  else {
    disp.drawStr(30, 8, "LOST");
  }

  if (imuWorking) {
    disp.drawStr(30, 63, "OK");
  }

  else {
    disp.drawStr(30, 63, "FAIL");
  }

  disp.sendBuffer();
}

bool radioUpdate(){

  if (radio.available()) {

    radio.read(&cr, sizeof(cr));

    radio.stopListening();
    ct.time = cr.time;
    radio.write(&ct, sizeof(ct));
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