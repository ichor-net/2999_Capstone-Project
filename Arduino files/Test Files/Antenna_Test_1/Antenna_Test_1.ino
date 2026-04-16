#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>

// ---------------- ROLE ----------------
// Set true on one board, false on the other
const bool IS_CONTROLLER = false;

// ---------------- NRF24 PINS ----------------
#define NRF_RX   0
#define NRF_CS   1
#define NRF_SCK  2
#define NRF_TX   3
#define NRF_CE   6

RF24 radio(NRF_CE, NRF_CS);

// Same addresses you've been using
byte txAddr[6] = "CNTRL";
byte rxAddr[6] = "PLANE";

// ---------------- TEST PACKETS ----------------
struct __attribute__((packed)) PacketTX {
  uint8_t seq;
  uint8_t magic;
};

struct __attribute__((packed)) PacketRX {
  uint8_t seq_echo;
  uint8_t magic_echo;
};

static_assert(sizeof(PacketTX) == 2, "PacketTX wrong size");
static_assert(sizeof(PacketRX) == 2, "PacketRX wrong size");

PacketTX tx = {0, 0x5A};
PacketRX rx = {0, 0};

uint32_t lastSend = 0;
uint32_t rxCount = 0;
uint32_t txCount = 0;

// ======================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(IS_CONTROLLER ? "ROLE: CONTROLLER" : "ROLE: PLANE");
  Serial.println("Starting radio-only test...");

  SPI.setRX(NRF_RX);
  SPI.setCS(NRF_CS);
  SPI.setSCK(NRF_SCK);
  SPI.setTX(NRF_TX);
  SPI.begin();

  bool ok = radio.begin();
  Serial.print("radio.begin(): ");
  Serial.println(ok ? "YES" : "NO");

  Serial.print("isChipConnected(): ");
  Serial.println(radio.isChipConnected() ? "YES" : "NO");

  if (!ok) {
    Serial.println("RADIO NOT FOUND");
    while (1) {
      delay(1000);
    }
  }

  // Debug-friendly settings first
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setRetries(3, 5);
  radio.setChannel(108);

  if (IS_CONTROLLER) {
    radio.openWritingPipe(txAddr);
    radio.openReadingPipe(1, rxAddr);
    radio.stopListening();
  } else {
    radio.openWritingPipe(rxAddr);
    radio.openReadingPipe(1, txAddr);
    radio.startListening();
  }

  radio.flush_rx();
  radio.flush_tx();

  Serial.println("Radio ready.");
}

// ======================================================
void loop() {
  if (IS_CONTROLLER) {
    controllerLoop();
  } else {
    planeLoop();
  }
}

// ======================================================
void controllerLoop() {
  uint32_t now = millis();
  if (now - lastSend < 100) return;   // 10 Hz test
  lastSend = now;

  tx.seq++;
  tx.magic = 0x5A;

  radio.stopListening();
  bool sent = radio.write(&tx, sizeof(tx));
  radio.startListening();

  txCount++;

  bool gotReply = false;
  PacketRX temp = {0, 0};

  uint32_t startWait = millis();
  while (millis() - startWait < 30) {
    if (radio.available()) {
      radio.read(&temp, sizeof(temp));
      gotReply = true;
      break;
    }
  }

  Serial.print("TX# ");
  Serial.print(txCount);
  Serial.print(" | sent=");
  Serial.print(sent ? "YES" : "NO");
  Serial.print(" | seq=");
  Serial.print(tx.seq);

  if (gotReply) {
    Serial.print(" | reply=YES");
    Serial.print(" | echo=");
    Serial.print(temp.seq_echo);
    Serial.print(" | magic=0x");
    Serial.print(temp.magic_echo, HEX);

    if (temp.seq_echo == tx.seq && temp.magic_echo == 0xA5) {
      Serial.print(" | MATCH");
    } else {
      Serial.print(" | BAD_REPLY");
    }
  } else {
    Serial.print(" | reply=NO");
  }

  Serial.println();
}

// ======================================================
void planeLoop() {
  if (!radio.available()) return;

  PacketTX in;
  while (radio.available()) {
    radio.read(&in, sizeof(in));
  }

  rxCount++;

  Serial.print("RX# ");
  Serial.print(rxCount);
  Serial.print(" | seq=");
  Serial.print(in.seq);
  Serial.print(" | magic=0x");
  Serial.println(in.magic, HEX);

  PacketRX out;
  out.seq_echo = in.seq;
  out.magic_echo = 0xA5;

  radio.stopListening();
  bool sent = radio.write(&out, sizeof(out));
  radio.startListening();

  Serial.print("Reply sent=");
  Serial.println(sent ? "YES" : "NO");
}