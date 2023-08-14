#include <AccelStepper.h>
#include "utils.h"

// Cart configurations
#define stepPin 3
#define dirPin 2
#define enablePin 8
#define MS1 7
#define MS2 6
#define MS3 5
#define homeSwitch 9

AccelStepper cart(AccelStepper::DRIVER, stepPin, dirPin);
void microStep(MICROSTEP mstep, int m1, int m2, int m3);
void cartCalibrationRoutine();

void setup()
{
  Serial.begin(115200);
  cartCalibrationRoutine();
}

void loop()
{
}

void cartCalibrationRoutine()
{
  microStep(SIXTEENTH, MS1, MS2, MS3);
  cart.move(-2000000);
  while (digitalRead(homeSwitch) == HIGH)
  {
    cart.run();
  }
  cart.setCurrentPosition(0);
}

void microStep(MICROSTEP mstep, int m1, int m2, int m3)
{
  if (FULL)
  {
    digitalWrite(m1, LOW);
    digitalWrite(m2, LOW);
    digitalWrite(m3, LOW);
  }
  else if (HALF)
  {
    digitalWrite(m1, HIGH);
    digitalWrite(m2, LOW);
    digitalWrite(m3, LOW);
  }
  else if (QUARTER)
  {
    digitalWrite(m1, LOW);
    digitalWrite(m2, HIGH);
    digitalWrite(m3, LOW);
  }
  else if (EIGHT)
  {
    digitalWrite(m1, HIGH);
    digitalWrite(m2, HIGH);
    digitalWrite(m3, LOW);
  }
  else if (SIXTEENTH)
  {
    digitalWrite(m1, HIGH);
    digitalWrite(m2, HIGH);
    digitalWrite(m3, HIGH);
  }
}