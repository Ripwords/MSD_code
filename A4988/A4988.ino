// Define pin connections & motor's steps per revolution
const int dirPin = 2;
const int stepPin = 3;
const int m3 = 5;
const int m2 = 6;
const int m1 = 7;
const int stepsPerRevolution = 200;

int microSteps;

void setup()
{
	// Declare pins as Outputs
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
  pinMode(m3, OUTPUT);
	digitalWrite(dirPin, LOW);
  microSteps = driverMicrostep("FULL");
}
void loop()
{

	// Spin motor quickly
	for(int x = 0; x < stepsPerRevolution * microSteps; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(500);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(500);
	}
}

int driverMicrostep(String step) {
  int microSteps;
  if (step == "FULL") {
    digitalWrite(m1, LOW);
    digitalWrite(m2, LOW);
    digitalWrite(m3, LOW);
    microSteps = 1;
  } else if (step == "HALF") {
    digitalWrite(m1, HIGH);
    digitalWrite(m2, LOW);
    digitalWrite(m3, LOW);
    microSteps = 2;
  } else if (step == "QUARTER") {
    digitalWrite(m1, LOW);
    digitalWrite(m2, HIGH);
    digitalWrite(m3, LOW);
    microSteps = 4;
  } else if (step == "EIGHT") {
    digitalWrite(m1, HIGH);
    digitalWrite(m2, HIGH);
    digitalWrite(m3, LOW);
    microSteps = 8;
  } else if (step == "SIXTEENTH") {
    digitalWrite(m1, HIGH);
    digitalWrite(m2, HIGH);
    digitalWrite(m3, HIGH);
    microSteps = 16;
  } else {
    microSteps = 0;
  }
  return microSteps;
}