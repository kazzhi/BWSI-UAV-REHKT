#include <Servo.h>

Servo latch;

void setup() {
  latch.attach(30);  // Attach latch servo to pin 30
  latch.write(8);    // Close latch (adjust angle if needed)
}

void loop() {
}
