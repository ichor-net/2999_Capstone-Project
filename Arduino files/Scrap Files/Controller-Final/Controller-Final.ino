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
#define BUTTON1 9
#define BUTTON2 10
#define ENC1 11
#define ENC2 12
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
  uint16_t servo_write[4] = {1500, 1500, 1500, 1000}; //port, starboard, elevator, motor
  uint8_t rtt_timer;
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

volatile Servo_Modifiers servoBuf[2];

volatile uint8_t writeIndex = 0;
volatile uint8_t readIndex  = 1;

void radioHandler();
void inputHandler(Packet_TX &p, Servo_Modifiers &c);
void controlHandler(int dir, int &index, uint8_t toggle, Servo_Modifiers &c_local);
int readToggleButton();
void draw(uint8_t index, uint8_t toggle, Servo_Modifiers c_local);


U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R2, U8X8_PIN_NONE);
RF24 radio(NRF_CE, NRF_CS);


byte txAddr[6] = "CNTRL";
byte rxAddr[6] = "PLANE";


void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting Controller.");
  multicore_reset_core1();
  delay(500);
  Serial.println("Core1 Reset.");
  multicore_launch_core1(core1_entry);
  delay(250);
  Serial.println("Core1 Launched");

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

  disp.begin();
  //disp.setFont(u8g2_font_5x7_tf);
  disp.setFont(u8g2_font_squeezed_r6_tr);
  disp.clearBuffer();
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

void core1_entry() {
  Wire.begin();

  disp.begin();
  disp.setFont(u8g2_font_5x7_tf);
  disp.setFontPosTop();
  disp.clearBuffer();

  RotaryEncoder enc(ENC1, ENC2, RotaryEncoder::LatchMode::TWO03);

  delay(1000);

  // --- timing ---
  const uint32_t DSP_PERIOD = 33; // ~30 FPS
  uint32_t lastTime = millis();
  int index = 0;
  int dir = 0;
  uint8_t toggle=0;

  Servo_Modifiers c_local;

  while (1) {
    uint32_t now = millis();
    enc.tick();
    dir = (int)(enc.getDirection());
    toggle = readToggleButton();
    controlHandler(dir, index, toggle, c_local);

    if (now - lastTime >= DSP_PERIOD) {
      lastTime = now;
      draw(index, toggle, c_local);
    }
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
void draw(uint8_t index, uint8_t toggle, Servo_Modifiers c_local) {
  disp.clearBuffer();

  disp.sendBuffer();
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