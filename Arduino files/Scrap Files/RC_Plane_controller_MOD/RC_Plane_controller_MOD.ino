#include <U8g2lib.h>
#include <Wire.h>
#include <RF24.h>
#include <SPI.h>
#include <FastLED.h>
#include <RotaryEncoder.h>

//input
#define JSTK_X 28
#define JSTK_Y 29
#define SLD_POT 27
#define BUTTON1 8
#define BUTTON2 26
#define ENC1 15
#define ENC2 14
//oled dsp
#define OLED_SDA 4
#define OLED_SCL 5

//nrf24l01
#define NRF_RX 0
#define NRF_CS 1
#define NRF_SCK 2
#define NRF_TX 3
#define NRF_CE 6

void inputHandler(Packet_TX &p, Servo_Modifiers &c);
void controlHandler(int dir, int &index, uint8_t toggle, Servo_Modifiers &c_local);
int readToggleButton();
void draw();
bool radioUpdate();

struct Packet_TX{
  uint16_t servo_write[4];
  uint8_t rtt_timer;
};

struct Packet_RX{
  uint8_t gyro_x;
  uint8_t gyro_y;
  uint8_t gyro_z;
  uint8_t rtt_timer;

};

struct Servo_Modifiers {
  union {
    struct {
      uint16_t servoTrim[4] = {1500, 1500, 1500, 1000};
      uint16_t servoRange[4] = {500, 500, 500, 500};
    };
    uint16_t access[8];
  };
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

Packet_TX GLOBAL_TX;
Packet_RX GLOBAL_RX;
Servo_Modifiers GLOBAL_MOD;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R0, U8X8_PIN_NONE);
RotaryEncoder enc(ENC1, ENC2, RotaryEncoder::LatchMode::TWO03);
RF24 radio(CE, CS);

void setup() {
  FastLED.addLeds<WS2812, 16, GRB>(led, 1);
  led[0] = CRGB(255-BRIGHTNESS, 255-BRIGHTNESS, 255-BRIGHTNESS);
  FastLED.show();

  Serial.begin(115200);
  delay(500);
  Serial.println("Starting Controller.");
  
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);

  SPI.setRX(NRF_RX);
  SPI.setCS(NRF_CS);
  SPI.setSCK(NRF_SCK);
  SPI.setTX(NRF_TX);
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

/*
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
*/

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

void inputHandler(Packet_TX &p, Servo_Modifiers &c) {
  const int min = 1000;
  const int max = 2000;

  int x = analogRead(JSTK_X);
  int y = analogRead(JSTK_Y);
  int t  = analogRead(SLD_POT);
  int temp[4];
  p.rtt_timer = millis() & 0xFF;

  temp[0] = (x*c.servoRange[0]/1024 - c.servoRange[0]/2 + c.servoTrim[0]);
  temp[1] = (x*c.servoRange[1]/1024 - c.servoRange[1]/2 + c.servoTrim[1]);
  temp[2] = (y*c.servoRange[2]/1024 - c.servoRange[2]/2 + c.servoTrim[2]);
  temp[3] = (t*c.servoRange[3]/1024 + c.servoTrim[3]);

  for (int k = 0; k<4; k++) {
    temp[k] = (temp[k] < min) ? min : (temp[k] > max) ? max : temp[k];
    p.servo_write[k] = temp[k];
  }
}

void controlHandler(int dir, int &index, uint8_t toggle, Servo_Modifiers &c_local) {
  const int bounds[2][2] = { {1000, 2000}, {0, 1000} };
  if (toggle) {
    int k = index >> 2;
    int temp = c_local.access[index];
    temp += dir*5;

    if (temp < bounds[k][0]) temp = bounds[k][0];
    else if (temp > bounds[k][1]) temp = bounds[k][1];
    c_local.access[index] = temp;  
  }

  else {
    index+=dir;
    if (index < 0) index = 0;
    else if (index > 7) index = 7;
  }
}

int readToggleButton() {
  static bool last = LOW;
  static int state = 0;

  bool now = digitalRead(BUTTON1);

  // falling edge = release (pulldown)
  if (last == HIGH && now == LOW) {
    state ^= 1;  // toggle 0 ↔ 1
  }

  last = now;
  return state;
}










