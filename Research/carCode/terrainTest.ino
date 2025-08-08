#include <Servo.h>

Servo leftWheel;
Servo rightWheel;

const int leftWheelPin = 3;
const int rightWheelPin = 2;

unsigned long startTime;

void setup() {
  leftWheel.attach(leftWheelPin);
  rightWheel.attach(rightWheelPin);
  startTime = millis();  // record the time the program started
}

void loop() {
  unsigned long now = millis();
  unsigned long elapsed = now - startTime;

  if (elapsed < 10000) {
    // Initial 10-second wait — do nothing
    leftWheel.write(90);
    rightWheel.write(90);
  } else if (elapsed < 12000) {
    // Drive forward for 5 seconds
    leftWheel.write(180);
    rightWheel.write(180);
  } else if (elapsed < 22000) {
    // Stop for 10 seconds
    leftWheel.write(90);
    rightWheel.write(90);
  } else if (elapsed < 24000) {
    // Turn left for 5 seconds
    leftWheel.write(0);     // full reverse
    rightWheel.write(180);  // full forward
  } else {
    // Stop forever
    leftWheel.write(90);
    rightWheel.write(90);
  }
}
