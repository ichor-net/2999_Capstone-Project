#include <Servo.h>

Servo s;
void setup() {
  s.attach(14);
  s.writeMicroseconds(1000);
}

void loop() {
static int last = millis();
int now = millis();

  if (now-last>=10) {
    int x = analogRead(27);
    x = map(x, 0, 1024, 1000, 2000);
    s.writeMicroseconds(x);
  }
}
