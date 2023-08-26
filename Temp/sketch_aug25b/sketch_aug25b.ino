#include <AccelStepper.h>

#define STEP 6
#define DIR 5
#define SIG 3
#define SW 2
#define MSTEP 4

AccelStepper cart(1, STEP, DIR);

void setup() {
  delay(2000);
  // put your setup code here, to run once:
  Serial.begin(115200);
  motorInit();
  digitalWrite(SIG, HIGH);
  delay(1000);
  digitalWrite(SIG, LOW);
  pinMode(SIG, INPUT);
  while(!digitalRead(SIG)) {;}
  cart.setAcceleration(50000);
  cart.move(-50 * MSTEP);
  moveTillEnd();
  cart.move(50 * MSTEP);
  cart.setAcceleration(10000);
  cart.move(-1470 * MSTEP);
  moveTillEnd();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void motorInit() {
  pinMode(SIG, OUTPUT);
  digitalWrite(SIG, LOW);
  cart.setAcceleration(10000);
  cart.setMaxSpeed(10000);

  cart.move(20000 * MSTEP);
  while (digitalRead(SW)) {
    cart.run();
  }
  cart.setCurrentPosition(0);
}

void moveTillEnd() {
  while (abs(cart.distanceToGo()) != 0) {
    cart.run();
  }
}