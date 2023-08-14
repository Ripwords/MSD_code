#ifndef A4988_H
#define A4988_H

enum Microstep
{
  FULL,
  HALF,
  QUARTER,
  EIGHT,
  SIXTEENTH
};

class A4988
{
public:
  // Constructor
  A4988(int stepPin, int dirPin, int enablePin, int PIN_M1, int PIN_M2, int PIN_M3) : stepPin(stepPin), dirPin(dirPin), enablePin(enablePin), PIN_M1(PIN_M1), PIN_M2(PIN_M2), PIN_M3(PIN_M3){};

  void begin()
  {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enablePin, OUTPUT);
    pinMode(PIN_M1, OUTPUT);
    pinMode(PIN_M2, OUTPUT);
    pinMode(PIN_M3, OUTPUT);

    digitalWrite(dirPin, LOW);
    digitalWrite(enablePin, LOW);
    setMicrosteps(FULL);
  }

  void setMicrosteps(Microstep mstep)
  {
    if (mstep == FULL)
    {
      digitalWrite(PIN_M1, LOW);
      digitalWrite(PIN_M2, LOW);
      digitalWrite(PIN_M3, LOW);
    }
    else if (mstep == HALF)
    {
      digitalWrite(PIN_M1, HIGH);
      digitalWrite(PIN_M2, LOW);
      digitalWrite(PIN_M3, LOW);
    }
    else if (mstep == QUARTER)
    {
      digitalWrite(PIN_M1, LOW);
      digitalWrite(PIN_M2, HIGH);
      digitalWrite(PIN_M3, LOW);
    }
    else if (mstep == EIGHT)
    {
      digitalWrite(PIN_M1, HIGH);
      digitalWrite(PIN_M2, HIGH);
      digitalWrite(PIN_M3, LOW);
    }
    else if (mstep == SIXTEENTH)
    {
      digitalWrite(PIN_M1, HIGH);
      digitalWrite(PIN_M2, HIGH);
      digitalWrite(PIN_M3, HIGH);
    }
  }

  void setDirection(int dir)
  {
    digitalWrite(dirPin, dir);
  }

  void setSpeed(double speed)
  {
    // Speed is in RPS
    // RPS
    // 1 / RPS = 1s / R
    // 1s / R / 360 = t per degree
    // t per degree * 1.8 = t per step
    stepDelay = pow(speed, -1) / 360.0 * 1.8;
    stepDelay *= 100000;
  }

  void step(int steps)
  {
    if (steps < 0)
    {
      digitalWrite(dirPin, HIGH);
      steps = -steps;
    }
    else
    {
      digitalWrite(dirPin, LOW);
    }
    // Complete stated number of steps with the step delay
    for (int x = 0; x < steps; x++)
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepDelay);
    }
  }

  void enable()
  {
    digitalWrite(enablePin, LOW);
  }

  void disable()
  {
    digitalWrite(enablePin, HIGH);
  }

private:
  int stepPin;
  int dirPin;
  int enablePin;
  int PIN_M1;
  int PIN_M2;
  int PIN_M3;
  double stepDelay;
};

#endif