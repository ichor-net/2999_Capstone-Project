#include <U8g2lib.h>
#include <Wire.h>
#include <RF24.h>
#include <SPI.h>
#include <FastLED.h>
#include <RotaryEncoder.h>
#include "pico/multicore.h"

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

struct Packet_TX{
  union {
    struct {
      uint16_t servo_write[4];
      uint8_t rtt_timer;
      uint8_t padding;   // force alignment
    };
    uint8_t access[10];
  };
};

struct Packet_RX{
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

struct Servo_Modifiers {
  union {
    struct {
      uint16_t servoTrim[4] = {1500, 1500, 1500, 1000};
      uint16_t servoRange[4] = {500, 500, 500, 500};
    };
    uint16_t access[8];
  };
};

void inputHandler(Packet_TX &p, Servo_Modifiers &c);
void radioHandler(Packet_TX &send, Packet_RX &receive);
void controlHandler(int dir, int &index, uint8_t toggle, Servo_Modifiers &c_local);
int readToggleButton();

Packet_TX GLOBAL_TX;
Packet_RX GLOBAL_RX;
Servo_Modifiers GLOBAL_MOD;

RotaryEncoder enc(ENC1, ENC2, RotaryEncoder::LatchMode::TWO03);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R2, U8X8_PIN_NONE);
RF24 radio(NRF_CE, NRF_CS);

byte txAddr[6] = "CNTRL";
byte rxAddr[6] = "PLANE";

uint32_t lastLinkTime=0;
bool linkOK=false;

const int BRIGHTNESS = 200;
CRGB led[1];

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
  radio.stopListening();

  Wire.begin();

  disp.begin();
  disp.setFont(u8g2_font_5x7_tf);
  disp.setFontPosTop();
  disp.clearBuffer();

}

void loop() {
    // --- .0f keeps it to integers, modify {[###].0f} for frequency adjustment (Hz)
  const uint32_t LOOP_HZ = 250.0f;
  const uint32_t LOOP_PERIOD_US = 1000000.0f / LOOP_HZ; 

  //Timers
  static uint32_t lastTime = micros();
  uint32_t now = micros();

  static int index = 0;
  static int dir = 0;
  static bool toggle=0;

  enc.tick();
  dir = (int)(enc.getDirection());
  toggle = readToggleButton();
  controlHandler(dir, index, toggle, GLOBAL_MOD);
  
  //Radio Loop
  if (now - lastTime >= LOOP_PERIOD_US) {
    lastTime = now;
    radioHandler(GLOBAL_TX, GLOBAL_RX);
  }

  /*
    if (now - lastTime >= DSP_PERIOD) {
      lastTime = now;
      draw(index, toggle, c_local);
    }
  */
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

void radioHandler(Packet_TX &send, Packet_RX &receive) {
  inputHandler(send, GLOBAL_MOD);
  radio.stopListening();
  bool success = radio.write(send.access, sizeof(send.access));
  radio.startListening();

  if (success) {
    lastLinkTime = millis();
    linkOK = true;
  }

  if (radio.available()) {
    radio.read(receive.access, sizeof(receive.access));
    receive.rtt_timer = (uint8_t)((millis() & 0xFF) - send.rtt_timer);
  }

  if (millis() - lastLinkTime > 100) {   // 100ms timeout
    linkOK = false;
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

void serialDraw() {

}