#include <AccelStepper.h>

// Pin Definitions
// Platform
#define DIR_PIN 4
#define PUL_PIN 3
// Dropper
#define DIR_PIN_2 6
#define PUL_PIN_2 5
// Screw
#define IN1 13
#define IN2 12
#define IN3 11
#define IN4 10
// Limit Switch
#define LIMIT 2

// SIGNAL CONTROL
#define SIG A0

AccelStepper platform(1, PUL_PIN, DIR_PIN); // DRIVER = 1
AccelStepper dropper(1, PUL_PIN_2, DIR_PIN_2);
AccelStepper screw(4, IN1, IN3, IN2, IN4); // FULL4WIRE = 4

void setup()
{
  Serial.begin(115200);
  motorInit();
  homePlatform();
}

void loop()
{
  while (!digitalRead(SIG))
  {
    ;
  }
  signalRecv();
}

void motorInit()
{
  pinMode(LIMIT, INPUT_PULLUP);
  pinMode(SIG, INPUT);

  platform.setMaxSpeed(2000);
  platform.setAcceleration(2000);

  screw.setMaxSpeed(400);
  screw.setAcceleration(75);

  dropper.setMaxSpeed(500);
  dropper.setAcceleration(200);
}

void homePlatform()
{
  Serial.println("Calibrating platform...");
  platform.move(2000000);
  while (digitalRead(LIMIT))
  {
    Serial.print("...");
    platform.run();
  }
  Serial.println("\nPlatform is homed");
  platform.setCurrentPosition(0);
}

void screwBolt()
{
  platform.move(-9000);
  screw.move(2038 * 5);
  dropper.move(round(100));
  bool screwDone = abs(dropper.distanceToGo()) == 0 && abs(platform.distanceToGo()) == 0 && abs(screw.distanceToGo()) == 0;
  while (!screwDone)
  {
    dropper.run();
    if (abs(dropper.distanceToGo() == 0))
    {
      platform.run();
    }
    if (abs(platform.distanceToGo()) < 3500)
    {
      screw.run();
    }
  }
}

void signalRecv()
{
  pinMode(SIG, OUTPUT);
  digitalWrite(SIG, LOW);
  delay(1000);

  screwBolt();

  digitalWrite(SIG, HIGH);
  delay(1000);
  digitalWrite(SIG, LOW);
  delay(1000);
  pinMode(SIG, INPUT);
}