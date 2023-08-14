// Code is split into regions for each stage of the Project
// Stage 1: Calibrate motors
// Stage 2: Move cart to the first station (Nut Slider 1)
// Stage 3: Move cart to the second station (Nut Slider 2)
// Stage 4: Move cart to the third station (Bottom Slider)
// Stage 5: Move cart to the fourth station (Top Slider)
// Stage 6: Part removal

#include <AccelStepper.h>

#define INTERFACE_TYPE 1

#define STEP_PIN 2
#define DIR_PIN 3
#define LIMIT 4

#define SIG_1 5
#define SIG_2 6
#define SIG_3 7
#define SIG_4 8
#define SIG_5 9
#define SIG_6 10

#define NUT_SLIDER_1 400
#define NUT_SLIDER_2 800
#define BOTTOM_SLIDER 1200
#define TOP_SLIDER 1600
#define PART_REMOVAL 2000

enum Stage
{
  STAGE_1,
  STAGE_2,
  STAGE_3,
  STAGE_4,
  STAGE_5,
  STAGE_6
};

// AccelStepper
AccelStepper cart(1, STEP_PIN, DIR_PIN);

void initMotors(int startAcc, int endAcc);
void moveToStage(int step);

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting...");
  initMotors(2000, 1000);
}

void loop()
{
  while (digitalRead(SIG_1) == HIGH)
  {
    moveToStage(NUT_SLIDER_1, "Nut Slider 1");
  }

  while (digitalRead(SIG_2) == HIGH)
  {
    stage2();
  }

  while (digitalRead(SIG_3) == HIGH)
  {
    stage3();
  }

  while (digitalRead(SIG_4) == HIGH)
  {
    stage4();
  }

  while (digitalRead(SIG_5) == HIGH)
  {
    stage5();
  }

  while (digitalRead(SIG_6) == HIGH)
  {
    stage6();
  }
}

void initMotors(int startAcc, int endAcc)
{
  cart.setMaxSpeed(1000);
  cart.setAcceleration(1500);

  Serial.println("Calibrating cart...");
  cart.move(-2000000);
  cart.setAcceleration(1000);
  while (!digitalRead(LIMIT))
  {
    Serial.print("... ");
    cart.run();
  }
  Serial.println("Cart is homed");
  cart.setCurrentPosition(0);
  cart.setAcceleration(1500);
}

void moveToStage(int step, string name = " ")
{
  Serial.println("Moving to " + name + "...");
  cart.move(1000000); // Set to the appropriate number of steps
  cart.setAcceleration(1000);
  while (cart.distanceToGo() != 0)
  {
    Serial.print("... ");
    cart.run();
  }
  Serial.println("Cart is at " + name);
}