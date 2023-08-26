#include <Servo.h>

#define SIG A0

#define NUT_1 11
#define NUT_2 10
#define BOTTOM 9
#define TOP 6
#define REMOVER 5

Servo bottomS;
Servo topS;
Servo removerS;
Servo nut1S;
Servo nut2S;

int currentStage = 0;

void setup()
{
  Serial.begin(115200);
  initSystem();
}

void loop()
{
  while (!digitalRead(SIG))
  {
    ;
  }
  signalRecv();
}

void initSystem()
{
  pinMode(SIG, INPUT);
  bottomS.attach(BOTTOM);
  topS.attach(TOP);
  removerS.attach(REMOVER);
  nut1S.attach(NUT_1);
  nut2S.attach(NUT_2);

  // Servo initial positions
  bottomS.write(180);
  topS.write(180);
  removerS.write(180);
  nut1S.write(180);
  nut2S.write(180);
}

void activateNut1()
{
}

void activateNut2()
{
}

void activateBottom()
{
  for (int i = 180; i >= 0; i--)
  {
    bottomServo.write(i);
    delay(10);
  }
  for (int i = 0; i <= 180; i++)
  {
    bottomServo.write(i);
    delay(10);
  }
}

void activateTop()
{
}

void activateRemover()
{
}

void stageController()
{
  switch (currentStage)
  {
  case 0:
    activateNut1();
    break;
  case 1:
    activateNut2();
    break;
  case 2:
    activateBottom();
    break;
  case 3:
    activateTop();
    break;
  case 4:
    activateRemover();
    break;
  default:
    break;
  }
  currentStage++;
  if (currentStage > 4)
  {
    currentStage = 0;
  }
}

void signalRecv()
{
  pinMode(SIG, OUTPUT);
  digitalWrite(SIG, LOW);
  delay(1000);

  stageController();

  digitalWrite(SIG, HIGH);
  delay(1000);
  digitalWrite(SIG, LOW);
  delay(1000);
  pinMode(SIG, INPUT);
}