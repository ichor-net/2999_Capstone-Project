#include <U8g2lib.h>
#include <Wire.h>
#include <RF24.h>
#include <SPI.h>

#define jx 27
#define jy 26
#define throttle 28
#define b1 6

#define OLED_SCL 5
#define OLED_SDA 4

#define RX 0
#define CS 1
#define SCK 2
#define TX 3
#define CE 8

void hwinput();
void draw();
bool radioUpdate();

struct Packet_TX{
  uint8_t x;
  uint8_t y;
  uint8_t t;
  uint8_t b;
  uint8_t time;
};

struct Packet_RX{
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
uint8_t rtt = 0;

byte txAddr[6] = "NODE1";
byte rxAddr[6] = "NODE2";

Packet_TX ct;
Packet_RX cr;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R0, U8X8_PIN_NONE);
RF24 radio(CE, CS);

void setup() {
  Serial.begin(115200);
  delay(1500);
  
  pinMode(throttle, INPUT);
  pinMode(jx, INPUT);
  pinMode(jy, INPUT);
  pinMode(b1, INPUT);

  SPI.setRX(RX);
  SPI.setCS(CS);
  SPI.setSCK(SCK);
  SPI.setTX(TX);
  SPI.begin();

  Wire.begin();   // use default SDA=4, SCL=5

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

  radio.stopListening();

  disp.begin();
  //disp.setFont(u8g2_font_5x7_tf);
  disp.setFont(u8g2_font_squeezed_r6_tr);
  disp.clearBuffer();
}

void loop() {

  static uint32_t lastRadio = 0;
  static uint32_t lastDisplay = 0;

  uint32_t now = millis();

  hwinput();

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

void hwinput() {
  static int sx = 512, sy = 512, st = 0;

  sx = (sx * 2 + analogRead(jx)) / 3;
  sy = (sy * 2 + analogRead(jy)) / 3;
  st = (st * 2 + analogRead(throttle)) / 3;

  ct.x = map(sx, 1023, 0, 0, 255);
  ct.y = map(sy, 1023, 0, 0, 255);
  ct.t = map(st, 0, 1023, 0, 255);
  ct.b = digitalRead(b1);
}

void draw() {
  disp.clearBuffer();

  disp.drawVLine(51, 6, 50);
  disp.drawDisc(51, 56 - map(ct.t, 0, 255, 0, 50), 2);

  disp.drawCircle(93, 31, 30);

  int px = map(ct.x, 0, 255, 63, 123);
  int py = map(ct.y, 0, 255, 61, 1);

  int dx = px - 93;
  int dy = py - 31;

  if (dx * dx + dy * dy > 30 * 30) {
    float a = atan2(dy, dx);
    px = 93 + cos(a) * 30;
    py = 31 + sin(a) * 30;
  }
  disp.drawDisc(px, py, 2);

  if (ct.b==0) {
    disp.drawCircle(61, 6, 4);
  }
  else {
    disp.drawDisc(61, 6, 3);
  }

  uint8_t *data = (uint8_t*)&cr;
  for (uint8_t i = 0; i < sizeof(cr)-1; i++) {
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
  char rttBuf[16];
  sprintf(rttBuf, "%u", rtt);
  disp.drawStr(30, 20, rttBuf);
  disp.sendBuffer();
}

bool radioUpdate(){

  radio.stopListening();
  ct.time = millis() & 0xFF;
  bool success = radio.write(&ct, sizeof(ct));

  radio.startListening();

  if (success) {
    lastLinkTime = millis();
    linkOK = true;
  }

  if (radio.available()) {
    radio.read(&cr, sizeof(cr));
    rtt = (uint8_t)((millis() & 0xFF) - cr.time);
  }

  if (millis() - lastLinkTime > 100) {   // 100ms timeout
    linkOK = false;
  }

  return success;
}