#include <U8g2lib.h>
#include <Wire.h>

#define jx 27
#define jy 26
#define throttle 28
#define b1 6

#define OLED_SCL 7
#define OLED_SDA 8

void hwinput();
void draw();

struct Control{
  int x;
  int y;
  int t;
};

Control c;
U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R0, U8X8_PIN_NONE);

void setup() {
  Serial.begin(9600);

  Wire.begin();   // use default SDA=4, SCL=5

  pinMode(throttle, INPUT);
  pinMode(jx, INPUT);
  pinMode(jy, INPUT);

  disp.begin();
  disp.setFont(u8g2_font_squeezed_r6_tr);
  disp.clearBuffer();
}

void loop() {
  hwinput();
  draw();
  delay(16);
}

void hwinput() {
  static int sx = 512, sy = 512, st = 0;

  sx = (sx * 3 + analogRead(jx)) / 4;
  sy = (sy * 3 + analogRead(jy)) / 4;
  st = (st * 3 + analogRead(throttle)) / 4;

  c.x = 1023 - sx;
  c.y = 1023 - sy;
  c.t = st;
}

void draw() {
  disp.clearBuffer();

  disp.drawVLine(8, 6, 50);
  disp.drawDisc(8, 56 - map(c.t, 0, 1023, 0, 50), 3);

  disp.drawCircle(93, 31, 30);

  int px = map(c.x, 0, 1023, 63, 123);
  int py = map(c.y, 0, 1023, 61, 1);

  int dx = px - 93;
  int dy = py - 31;

  if (dx * dx + dy * dy > 30 * 30) {
    float a = atan2(dy, dx);
    px = 93 + cos(a) * 30;
    py = 31 + sin(a) * 30;
  }

  disp.drawDisc(px, py, 2);
  disp.sendBuffer();
}