#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <RF24.h>
#include <SPI.h>
#include <FastLED.h>

// ---------------- INPUT ----------------
#define JSTK_X   29
#define JSTK_Y   28
#define SLD_POT  27
#define BUTTON1  8
#define BUTTON2  26
#define ENC1     15
#define ENC2     14

// ---------------- OLED ----------------
#define OLED_SDA 4
#define OLED_SCL 5

// ---------------- NRF24 ----------------
#define NRF_RX   0
#define NRF_CS   1
#define NRF_SCK  2
#define NRF_TX   3
#define NRF_CE   6

// ---------------- LED ----------------
#define LED_PIN 16
const int BRIGHTNESS = 120;
CRGB led[1];

// ---------------- SERVO ORDER ----------------
// 0 = port
// 1 = starboard
// 2 = elevator
// 3 = throttle

// ---------------- HARDCODED SERVO SETTINGS ----------------
// Adjust these directly if you want trim/range changes.
const uint16_t SERVO_TRIM_US[4]  = {1500, 1500, 1500, 1000};
const uint16_t SERVO_RANGE_US[4] = {500, 500, 500, 1000};

// ---------------- DATA ----------------
// rtt_timer is being used as a packet ID / echo tag.
struct __attribute__((packed)) Packet_TX {
  uint16_t servo_write[4];
  uint8_t rtt_timer;
};

struct __attribute__((packed)) Packet_RX {
  uint8_t gyro_x;
  uint8_t gyro_y;
  uint8_t gyro_z;
  uint8_t rtt_timer;
};

static_assert(sizeof(Packet_TX) == 9, "Packet_TX size wrong");
static_assert(sizeof(Packet_RX) == 4, "Packet_RX size wrong");

// ---------------- GLOBALS ----------------
Packet_TX GLOBAL_TX = {{1500, 1500, 1500, 1000}, 0};
Packet_RX GLOBAL_RX = {128, 128, 128, 0};

U8G2_SSD1306_128X64_NONAME_F_HW_I2C disp(U8G2_R2, U8X8_PIN_NONE);
RF24 radio(NRF_CE, NRF_CS);

byte txAddr[6] = "CNTRL";
byte rxAddr[6] = "PLANE";

uint32_t lastLinkTime = 0;
bool linkOK = false;
uint8_t nextPacketId = 0;
uint8_t awaitedPacketId = 0;
uint32_t replyCount = 0;

// ---------------- PROTOTYPES ----------------
void inputHandler(Packet_TX &p);
bool radioHandler(Packet_TX &send, Packet_RX &receive);
void debugOutput(const Packet_TX &p, const Packet_RX &r, bool gotReply);

// ======================================================
void setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(led, 1);
  led[0] = CRGB(255 - BRIGHTNESS, 255 - BRIGHTNESS, 255 - BRIGHTNESS);
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

  // Same settings that matched your working test
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.setChannel(108);

  radio.openWritingPipe(txAddr);
  radio.openReadingPipe(1, rxAddr);
  radio.stopListening();
  radio.flush_rx();
  radio.flush_tx();

  Wire.setSDA(OLED_SDA);
  Wire.setSCL(OLED_SCL);
  Wire.begin();

  disp.begin();
  disp.setFont(u8g2_font_5x7_tf);
  disp.setFontPosTop();
  disp.clearBuffer();
}

// ======================================================
void loop() {
  static uint32_t lastTime = 0;
  uint32_t now = millis();

  if (now - lastTime >= 10) {   // 100 Hz
    lastTime += 10;
    radioHandler(GLOBAL_TX, GLOBAL_RX);
  }
}

// ======================================================
void inputHandler(Packet_TX &p) {
  static int sx = 512;
  static int sy = 512;
  static int st = 0;

  // 3-tick smoothing on raw analogs
  sx = (sx * 2 + analogRead(JSTK_X)) / 3;
  sy = (sy * 2 + analogRead(JSTK_Y)) / 3;
  st = (st * 2 + analogRead(SLD_POT)) / 3;

  int x0 = (sx * SERVO_RANGE_US[0]) / 1024 - (SERVO_RANGE_US[0] / 2);
  int x1 = (sx * SERVO_RANGE_US[1]) / 1024 - (SERVO_RANGE_US[1] / 2);
  int y2 = (sy * SERVO_RANGE_US[2]) / 1024 - (SERVO_RANGE_US[2] / 2);
  int t3 = (st * SERVO_RANGE_US[3]) / 1024;

  p.servo_write[0] = constrain(SERVO_TRIM_US[0] + x0, 1000, 2000);
  p.servo_write[1] = constrain(SERVO_TRIM_US[1] + x1, 1000, 2000);
  p.servo_write[2] = constrain(SERVO_TRIM_US[2] + y2, 1000, 2000);
  p.servo_write[3] = constrain(SERVO_TRIM_US[3] + t3, 1000, 2000);

  // packet ID
  p.rtt_timer = ++nextPacketId;
}

// ======================================================
bool radioHandler(Packet_TX &send, Packet_RX &receive) {
  inputHandler(send);
  awaitedPacketId = send.rtt_timer;

  radio.stopListening();
  radio.write(&send, sizeof(send));
  radio.startListening();

  bool gotReply = false;
  uint32_t waitStart = millis();

  // Since the radio-only test worked, wait briefly for matching echo
  while (millis() - waitStart < 30) {
    if (radio.available()) {
      Packet_RX temp;
      radio.read(&temp, sizeof(temp));

      if (temp.rtt_timer == awaitedPacketId) {
        receive = temp;
        lastLinkTime = millis();
        linkOK = true;
        gotReply = true;
        replyCount++;
        break;
      }
    }
  }

  if (millis() - lastLinkTime > 100) {
    linkOK = false;
  }

  debugOutput(send, receive, gotReply);

  // Green only when actually receiving matching plane data
  if (gotReply) {
    led[0] = CRGB(0, 255 - BRIGHTNESS, 0);
  } else {
    led[0] = CRGB(255 - BRIGHTNESS, 0, 0);
  }

  FastLED.show();
  return gotReply;
}

// ======================================================
void debugOutput(const Packet_TX &p, const Packet_RX &r, bool gotReply) {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();

  if (now - lastPrint < 100) return;
  lastPrint = now;

  Serial.print("RX_OK: ");
  Serial.print(gotReply ? "YES" : "NO");

  Serial.print(" | LINK: ");
  Serial.print(linkOK ? "OK" : "LOST");

  Serial.print(" | TX_ID: ");
  Serial.print(p.rtt_timer);

  Serial.print(" | WAIT_ID: ");
  Serial.print(awaitedPacketId);

  Serial.print(" | ECHO_ID: ");
  Serial.print(r.rtt_timer);

  Serial.print(" | REPLIES: ");
  Serial.print(replyCount);

  Serial.print(" | SERVO: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(p.servo_write[i]);
    if (i < 3) Serial.print(", ");
  }

  Serial.print(" | GYRO: ");
  Serial.print(r.gyro_x);
  Serial.print(", ");
  Serial.print(r.gyro_y);
  Serial.print(", ");
  Serial.print(r.gyro_z);

  Serial.println();
}