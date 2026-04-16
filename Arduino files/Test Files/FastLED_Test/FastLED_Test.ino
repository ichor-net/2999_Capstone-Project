#include <Servo.h>
#include <FastLED.h>
Servo s;

CRGB led[1];

void setup() {
  s.attach(0);
  FastLED.addLeds<WS2812, 16, GRB>(led, 1);
}

void loop() {
  static int time=millis();
  int now=millis();
  int a = analogRead(27);
  int ms = map(a, 0, 1023, 1000, 2000);
  int c = a >> 2;
  led[0] = CRGB(c, c, c);
  if (now-time >= 8) {
    s.writeMicroseconds(ms);
    FastLED.show();
  }
}
