#include <U8g2lib.h>
#include <Wire.h>
#include <RF24.h>
#include <SPI.h>
#include <FastLED.h>
#include <RotaryEncoder.h>
#include "pico/multicore.h"

//input
#define JSTK_X 28
#define JSTK_Y 27
#define SLD_POT 29
#define BUTTON 11
#define ENC1 5
#define ENC2 6
//oled dsp
#define OLED_SDA 15
#define OLED_CS 7
#define OLED_SCL 14
#define OLED_DC 26
#define OLED_RST 8
//nrf24l01
#define NRF_RX 0
#define NRF_CS 1
#define NRF_SCK 2
#define NRF_TX 3
#define NRF_CE 4

struct Packet_TX{
  union {
    struct {
      uint16_t servo_write[4]; //port, starboard, elevator, motor
      uint16_t rtt_timer;
    };
    uint16_t access[5];
  };
};

struct Packet_RX{
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

struct Control_0{
  uint32_t lastLinkTime = 0;
  bool linkOK = false;
  uint16_t jx;
  uint16_t jy;
  uint16_t t;
  uint16_t deadzone = 20;
  uint16_t ctrl_mid = 512;
};

struct Control_1{
  uint8_t button;
  uint16_t trim[4]={1500, 1500, 1500, 1000}; //port, starboard, elevator, motor
  uint16_t range[4]={500, 500, 500, 500}; //port, starboard, elevator, motor
};

const char *names[] = {
  "GX:",
  "GY:",
  "GZ:",
  "AX:",
  "AY:",
  "AZ:"
};

void core1_entry();
void dataLoader(Packet_TX &p, Control &c);
void controlSet(Control &c);
void radioHandler(Packet_TX &send, Packet_RX &receive, Control &c_local);
void draw(const Packet_TX &tx_draw, const Packet_RX &rx_draw, const Control &c_draw);

byte txAddr[6] = "PLANE";
byte rxAddr[6] = "CNTRL";
CRGB led[1];

Packet_TX tx;
Packet_RX rx;
Control c_mod;

Packet_TX tx_buf[2];
Packet_RX rx_buf[2];
Control c_buf[2];
const int BRIGHTNESS = 200;

volatile uint8_t tx_idx = 0;
volatile uint8_t rx_idx = 0;
volatile uint8_t c_idx = 0;
volatile bool core1_started = false;

U8G2_SH1122_256X64_F_2ND_4W_HW_SPI disp(U8G2_R0, OLED_CS, OLED_DC, OLED_RST);
RF24 radio(NRF_CE, NRF_CS);
RotaryEncoder enc(ENC1, ENC2, RotaryEncoder::LatchMode::TWO03);

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing.");

  multicore_reset_core1();

  FastLED.addLeds<WS2812, 16, GRB>(led, 1);
  led[0] = CRGB(255-BRIGHTNESS, 255-BRIGHTNESS, 255-BRIGHTNESS);
  FastLED.show();
  delay(1000);

  //Initializing Core0 SPI Channel for Radio------
  Serial.println("Starting SPI0.");
  SPI.setRX(NRF_RX);
  SPI.setCS(NRF_CS);
  SPI.setSCK(NRF_SCK);
  SPI.setTX(NRF_TX);
  SPI.begin();

  Serial.println("SPI0 Started.");
  led[0] = CRGB(255-BRIGHTNESS, 255-BRIGHTNESS, 0);
  FastLED.show();
  delay(1000);
  //-----------------------------------------------

  //Radio Failsafe----------------------
  Serial.println("Starting Radio...");
  if (!radio.begin()){ 
    Serial.println("Radio Failed!");
    led[0] = CRGB(255-BRIGHTNESS, 0, 0);
    FastLED.show();
    while(!radio.begin()) {
      Serial.println("Radio Failed!");
      delay(250);
    }
  }
  //------------------------------------

  //Radio Settings---------------------------------
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS);
  radio.setRetries(0,0);
  radio.setChannel(108);
  radio.enableAckPayload();
  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1, rxAddr);
  radio.startListening();

  Serial.println("Radio Started.");
  led[0] = CRGB(255-BRIGHTNESS, 0, 255-BRIGHTNESS);
  FastLED.show();
  delay(1000);


  multicore_launch_core1(core1_entry);
  delay(250);
  (core1_started) ? Serial.println("Core1 Launched.") : Serial.println("Core1 Failed to Launch.");

  led[0] = CRGB(0, 255-BRIGHTNESS, 0);
  FastLED.show();

  Serial.println("Initializer Complete!");
}



void loop() {
  // --- .0f keeps it to integers, modify {[###].0f} for frequency adjustment (Hz)
  const uint32_t LOOP_HZ = 250.0f;
  const uint32_t LOOP_PERIOD_US = 1000000.0f / LOOP_HZ; 

  //Timers
  static uint32_t lastTime = micros();
  uint32_t now = micros();

  //Radio Loop and memory management
  if (now - lastTime >= LOOP_PERIOD_US) {
    lastTime = now;

    radioHandler(tx, rx, c_mod);

    uint8_t tx_next = tx_idx ^ 1;
    tx_buf[tx_next] = tx;
    __dmb();
    tx_idx = tx_next;

    uint8_t rx_next = rx_idx ^ 1;
    rx_buf[rx_next] = rx;
    __dmb();
    rx_idx = rx_next;

    uint8_t c_next = c_idx ^ 1;
    c_buf[c_next] = c_mod;
    __dmb();
    c_idx = c_next;
  }

}

void dataLoader(Packet_TX &p, Control &c) {
  uint16_t x = analogRead(JSTK_X);
  uint16_t y = analogRead(JSTK_Y);
  uint16_t t  = analogRead(SLD_POT);
  p.rtt_timer = millis() & 0xFF;

  p.servo_write[0] = ( (( ((c.ctrl_mid - c.deadzone) < x) && (x < (c.ctrl_mid + c.deadzone)) ) ? 512 : x ) * c.range[0] / 1024) - (c.range[0] / 2) + c.trim[0];
  p.servo_write[1] = ( (( ((c.ctrl_mid - c.deadzone) < x) && (x < (c.ctrl_mid + c.deadzone)) ) ? 512 : x ) * c.range[1] / 1024) - (c.range[1] / 2) + c.trim[1];
  p.servo_write[2] = ( (( ((c.ctrl_mid - c.deadzone) < y) && (y < (c.ctrl_mid + c.deadzone)) ) ? 512 : y ) * c.range[2] / 1024) - (c.range[2] / 2) + c.trim[2];
  p.servo_write[3] = (t * c.range[3] / 1024) + c.trim[3];

  c.jx = (x >= 1024) ? 255 : (x >> 2);
  c.jy = (y >= 1024) ? 255 : (y >> 2);
  c.t = (t >= 1024) ? 255 : (t >> 2);
}

void radioHandler(Packet_TX &send, Packet_RX &receive, Control &c_local) {
  dataLoader(send, c_local);
  radio.stopListening();
  bool success = radio.write(send.access, sizeof(send.access));
  radio.startListening();

  if (success) {
    c_local.lastLinkTime = millis();
    c_local.linkOK = true;
  }

  if (radio.available()) {
    radio.read(receive.access, sizeof(receive.access));
    receive.rtt_timer = (uint8_t)((millis() & 0xFF) - send.rtt_timer);
  }

  if (millis() - c_local.lastLinkTime > 100) {   // 100ms timeout
    c_local.linkOK = false;
  }
}

void core1_entry() {
  core1_started=true;
  SPI1.setTX(OLED_SDA);
  SPI1.setCS(OLED_CS);
  SPI1.setSCK(OLED_SCL);
  SPI1.begin();

  disp.begin();
  disp.setFont(u8g2_font_5x7_tf);
  //disp.setFont(u8g2_font_squeezed_r6_tr);
  disp.setFontPosTop();
  disp.setBusClock(8000000);
  disp.clearBuffer();
  
  const int DSP_FPS = 30.0f;
  const int DSP_PERIOD = 1000.0f/DSP_FPS;

  Packet_RX rx_local;
  Packet_TX tx_local;
  Control c_local;

  static uint32_t lastTime = millis();
  uint32_t now = millis();

  while (1) {
    now = millis();

    if (now - lastTime >= DSP_PERIOD) {
      lastTime = now;

      // Loading TX from buffer
      uint8_t tx_read = tx_idx;
      __dmb();
      tx_local = tx_buf[tx_read];
      __dmb();

      //Loading RX from buffer
      uint8_t rx_read = rx_idx;
      __dmb();
      rx_local = rx_buf[rx_read];
      __dmb();

      //Loading Control from buffer
      uint8_t idx = c_idx;
      __dmb();
      c_local = c_buf[idx];
      __dmb();

      draw(tx_local, rx_local, c_local);
    }
  }
}

void draw(const Packet_TX &tx_draw, const Packet_RX &rx_draw, const Control &c_draw) {
  const int OFFSET = 64;
  disp.clearBuffer();
  
  disp.sendBuffer();
}

void drawJstk(const Control &c_draw) {
  disp.drawCircle(93+OFFSET, 31, 30);

  int px = map(c_draw.jx, 0, 255, 63+OFFSET, 123+OFFSET);
  int py = map(c_draw.jy, 0, 255, 61, 1);

  int dx = px - (93+OFFSET);
  int dy = py - 31;

  if (dx * dx + dy * dy > 30 * 30) {
    float a = atan2(dy, dx);
    px = (93+OFFSET) + cos(a) * 30;
    py = 31 + sin(a) * 30;
  }
  disp.drawDisc(px, py, 2);

}

void drawThrottle(const uint16_t t) {
  disp.drawVLine(51+OFFSET, 6, 50);
  disp.drawDisc(51+OFFSET, 56 - map(t, 0, 255, 0, 50), 2);

}

void controlMod() {

}

void controlDraw() {
  if (toggle)
}

void drawRX(const Packet_RX &rx_draw, const Control &c_draw) {
    for (uint8_t i = 0; i < sizeof(rx_draw.access)-1; i++) {
    char buf[16];
    sprintf(buf, "%s %u", names[i], rx_draw.access[i]);

    disp.drawStr(0, 10 + i*10, buf);
  }

  if (c_draw.linkOK) {
    char rttBuf[16];
    sprintf(rttBuf, "%u", rx_draw.rtt_timer);
    disp.drawStr(30, 20, rttBuf);
    disp.drawStr(30, 8, "LINK");
  }
  else {
    disp.drawStr(30, 8, "LOST");
    disp.drawStr(30, 20, "0");
  }
}