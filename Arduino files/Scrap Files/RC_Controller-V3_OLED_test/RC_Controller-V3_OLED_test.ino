#include <U8g2lib.h>
#include <SPI.h>
#include <RotaryEncoder.h>
#include "pico/multicore.h"

#define SPI SPI1
//oled dsp
#define OLED_SDA 15
#define OLED_CS 7
#define OLED_SCL 14
#define OLED_DC 26
#define OLED_RST 8
//input
#define JSTK_X 28
#define JSTK_Y 27
#define SLD_POT 29
#define BUTTON 11
#define ENC1 2
#define ENC2 3

struct core0_data {
  uint16_t jx;
  uint16_t jy;
};

struct core1_data {
  int8_t pos;
  uint8_t sw;
};

void draw(core1_data &c);

core0_data cd0;
core1_data cd1;
volatile int8_t GPOS=0;

U8G2_SH1122_256X64_F_4W_HW_SPI disp(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);
/*
U8G2_SH1122_256X64_2_4W_SW_SPI disp(
  U8G2_R0, OLED_SCL, OLED_SDA, OLED_CS, OLED_DC, OLED_RST
);
*/
volatile bool core1_started = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  multicore_reset_core1();
  Serial.println("Core1 Reset.");


  multicore_launch_core1(core1_entry);
  delay(250);
  (core1_started) ? Serial.println("Core1 Launched.") : Serial.println("Core1 Failed to Launch.");

  Serial.println("Initializer Complete!");
}

void loop() {
  static uint32_t lastTime = millis();
  uint32_t now = millis();

  if (now - lastTime >= 30) {
    lastTime=now;
    int8_t c = GPOS;
    Serial.println(c);
  }
}

void core1_entry() {
  core1_started = true;

  // --- SPI1 setup ---
  SPI.setTX(OLED_SDA);
  SPI.setSCK(OLED_SCL);
  SPI.begin();

  // --- encoder ---
  RotaryEncoder enc(ENC1, ENC2, RotaryEncoder::LatchMode::TWO03);

  // --- display reset ---
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, LOW);
  delay(20);
  digitalWrite(OLED_RST, HIGH);
  delay(20);

  // --- display init ---
  disp.begin();
  disp.setBusClock(2000000);  // start safe (can increase later)
  disp.setFont(u8g2_font_5x7_tf);
  disp.setFontPosTop();

  // 🔥 TEST DRAW (IMPORTANT)
  disp.clearBuffer();
  disp.drawStr(0, 0, "INIT OK");
  disp.sendBuffer();

  delay(1000);

  // --- timing ---
  const uint32_t DSP_PERIOD = 33; // ~30 FPS
  uint32_t lastTime = millis();

  while (1) {
    uint32_t now = millis();

    enc.tick();
    int dir = (int)(enc.getDirection());

    if (dir != 0) {
      cd1.pos += dir;

      if (cd1.pos < 0) cd1.pos = 0;
      if (cd1.pos > 7) cd1.pos = 7;

      GPOS = cd1.pos;
    }

    if (now - lastTime >= DSP_PERIOD) {
      lastTime = now;
      draw(cd1);
    }
  }
}

void draw(core1_data &c) {
  disp.clearBuffer();
  char buf[8];
  itoa(c.pos, buf, 8);
  disp.drawStr(10, 10, buf);
  disp.sendBuffer();
}