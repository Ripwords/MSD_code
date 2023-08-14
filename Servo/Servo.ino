#include <Servo.h>

Servo servo_1; // servo controller (multiple can exist)

int servo_pin = 9; // PWM pin for servo control
int pos = 0;    // servo starting position

void setup() {
  servo_1.attach(servo_pin); // start servo control
}

void loop() {
  for (int pos = 0; pos <= 160; pos++) {
    servo_1.write(pos);
    delay(10);
  }
  for (int pos = 159; pos > 0; pos--) {
    servo_1.write(pos);
    delay(10);
  }
}