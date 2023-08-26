#include <Servo.h>

#define controlSig 5

Servo bottomServo; // create servo object to control a servo

int pos = 0; // variable to store the servo position

void setup()
{
  pinMode(controlSig, INPUT);
  bottomServo.attach(9); // attaches the servo on pin 9 to the servo object
  bottomServo.write(180);
  bottomBase();
}

void loop()
{
}

void bottomBase() {
  while(!digitalRead(controlSig)) {;}
  pinMode(controlSig, OUTPUT);
  pinMode(controlSig, LOW);
  delay(2000);

  for (int pos = 180; pos >= 0; pos--) {
    bottomServo.write(pos);
    delay(10);
  }
  for (int pos = 0; pos <= 180; pos++) {
    bottomServo.write(pos);
    delay(10);
  }
  
  digitalWrite(controlSig, HIGH);
  delay(1000);
  digitalWrite(controlSig, LOW);
  delay(1000);
  pinMode(controlSig, INPUT);
}