// Code is split into regions for each stage of the Project
// Stage 1: Calibrate motors
// Stage 2: Move cart to the first station (Nut Slider 1)
// Stage 3: Move cart to the second station (Nut Slider 2)
// Stage 4: Move cart to the third station (Bottom Slider)
// Stage 5: Move cart to the fourth station (Top Slider)
// Stage 6: Bolt 1
// Stage 7: Bolt 2
// Stage 8: Part removal

#include <AccelStepper.h>

#define INTERFACE_TYPE 1

#define STEP_PIN 13
#define DIR_PIN 12
#define LIMIT 11

#define SIG_1 A0
#define SIG_2 A0
#define SIG_3 A0
#define SIG_4 A0
#define SIG_5 A1
#define SIG_6 A2
#define SIG_7 A0

// Station positions
#define NUT_SLIDER_1 400
#define NUT_SLIDER_2 800
#define BOTTOM_SLIDER 1200
#define TOP_SLIDER 1600
#define BOLT_2 2000
#define PART_REMOVAL 1450

// System Configuration
#define PATH_MAX 1450
#define M_STEP 4

int singals[7] = {SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7};

// AccelStepper
AccelStepper cart(1, STEP_PIN, DIR_PIN);

void setup()
{
  Serial.begin(115200);

  // STAGE 1
  initSystem();
  initMotors();
}

void loop()
{
  // STAGE 2: Move cart to the first station (Nut Slider 1)
  moveToStage(NUT_SLIDER_1, SIG_1);

  // STAGE 3: Move cart to the second station (Nut Slider 2)
  moveToStage(NUT_SLIDER_2, SIG_2);

  // STAGE 4: Move cart to the third station (Bottom Slider)
  moveToStage(BOTTOM_SLIDER, SIG_3);

  // STAGE 5: Move cart to the fourth station (Top Slider)
  moveToStage(TOP_SLIDER, SIG_4);

  // STAGE 6: Bolt 1
  // Bolt 1 station (does not requirement cart movement)
  moveToStage(0, SIG_5);

  // STAGE 7: Bolt 2
  // Bolt 2 station
  moveToStage(BOLT_2, SIG_6);

  // STAGE 8: Part removal
  // Part removal station (does not requirement cart movement)
  moveToStage(PART_REMOVAL, SIG_5);

  // Return to home position
  initMotors();
}

void initSystem()
{
  for (int i = 0; i < 7; i++)
  {
    pinMode(singals[i], OUTPUT);
    digitalWrite(singals[i], LOW);
  }
  delay(1000);
}

void initMotors()
{
  cart.setMaxSpeed(10000);
  cart.setAcceleration(10000);

  Serial.println("Calibrating cart...");
  cart.move(2000000 * M_STEP);
  while (digitalRead(LIMIT))
  {
    Serial.print(".");
    cart.run();
  }
  Serial.println("Cart is homed");
  cart.setCurrentPosition(0);
}

void moveToStage(int steps, int signal)
{
  cart.move(steps * M_STEP);
  moveTillEnd();
  activateStage(signal);
}

void activateStage(int signal)
{
  digitalWrite(signal, HIGH);
  delay(1000);
  digitalWrite(signal, LOW);
  pinMode(signal, INPUT);

  while (!digitalRead(signal))
  {
    ;
  }

  pinMode(signal, OUTPUT);
  delay(1000);
}

void vibrateCart()
{
  cart.setAcceleration(50000);
  cart.move(-50 * M_STEP);
  moveTillEnd();
  cart.move(50 * M_STEP);
  cart.setAcceleration(10000);
}

void moveTillEnd()
{
  while (abs(cart.distanceToGo()) != 0)
  {
    cart.run();
  }
}