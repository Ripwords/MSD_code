#include <Servo.h>

Servo edgeServo; // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0; // variable to store the servo position

void setup()
{
  edgeServo.attach(9); // attaches the servo on pin 9 to the servo object
}

void loop()
{
  for (pos = 0; pos <= 140; pos += 1)
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    edgeServo.write(pos); // tell servo to go to position in variable 'pos'
    delay(15);            // waits 15ms for the servo to reach the position
  }
  for (pos = 140; pos <= 180; pos += 20) //ending pos
  { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    edgeServo.write(pos); // tell servo to go to position in variable 'pos'
    delay(1);             // waits 15ms for the servo to reach the position
  }
  delay(350); //150

  for (pos = 180; pos >= 140; pos -= 20) //default 10
  {                       // goes from 180 degrees to 0 degrees Push
    edgeServo.write(pos); // tell servo to go to position in variable 'pos'
    delay(1);             // waits 15ms for the servo to reach the position
  }
  for (pos = 140; pos >= 60; pos -= 1)
  {                       // goes from 180 degrees to 0 degrees Push
    edgeServo.write(pos); // tell servo to go to position in variable 'pos'
    delay(15);            // waits 15ms for the servo to reach the position
  }
  for (pos = 60; pos >= 0; pos -=20) {
    edgeServo.write(pos);
    delay(1);
  }
}