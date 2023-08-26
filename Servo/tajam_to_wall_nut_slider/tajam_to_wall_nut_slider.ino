#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  delay(5000);
  for (pos = 0; pos <= 40; pos += 20) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(1);                       // waits 15ms for the servo to reach the position
  }
    for (pos = 40; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }

  for (pos = 180; pos >= 40; pos -= 1) { // goes from 180 degrees to 0 degrees Push
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 40; pos >= 0; pos -= 20) { // goes from 180 degrees to 0 degrees Push
  myservo.write(pos);              // tell servo to go to position in variable 'pos'
  delay(1);                       // waits 15ms for the servo to reach the position
  }
}