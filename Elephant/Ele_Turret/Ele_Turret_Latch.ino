#include <esp_now.h>
#include <WiFi.h>
#include <AccelStepper.h>
#include <Adafruit_MCP23X17.h>
#include "esp_private/wifi.h"

uint8_t broadcastAdd[] = { 0x0C, 0xB8, 0x15, 0xF8, 0xA1, 0x24 };

#define motorInterfaceType 1

// Turntable
#define TURN_DIR 14
#define TURN_PWM 16

// Linear Actuator
#define LIN_ACT_PWM 4
#define LIN_ACT_DIR 17

// Flywheel
#define FLY_LEFT_PWM 5
#define FLY_RIGHT_PWM 2
#define FLY_LEFT_DIR 18
#define FLY_RIGHT_DIR 33

// Reload
#define RELOAD_DIR 13
#define RELOAD_PWM 19
#define RELOAD_SW1 15
#define RELOAD_SW2 25


// Shooting
#define SHOOT_DIR 23 
#define SHOOT_PWM 32
#define SHOOT_SW 27

// Potentiometer
#define TUR_POT 35

// Solenoid
#define P_1 0
#define P_1 1

// Variable 
int reloadDir = 1;
double cubeAngle;

// Turntable Variables
AccelStepper turntable(motorInterfaceType, TURN_PWM, TURN_DIR);
AccelStepper shootMotor(motorInterfaceType, SHOOT_PWM, SHOOT_DIR);
AccelStepper reloadMotor(motorInterfaceType, RELOAD_PWM, RELOAD_DIR);

// Communication Data Structure
typedef struct robot_control {
  // Robot Movement
  // F/R (Front/Rear)
  // L/R (Left/Right)
  //   W (Wheel)
  float FLW, FRW, RLW, RRW;

  // Robot Actions Control
  // Turret Angle, Flywheel Speed, Reload, Shoot, Turntable (optional)
  // float turretAng, flywheel, reloadTrg, shootTrg, turnTbl;
  float turretAng, flywheel, turnTbl, reloadTrg, shootTrg;

  // Special Functions
  bool shootSequence, ring_1, ring_2;
} robot_control;

// Latch to skip function runs
float tAng, fwSpd, tTbl;

typedef struct pot_ang {
  int pot;
} pot_ang;

robot_control robCon;
pot_ang potAng;
Adafruit_MCP23X17 MCP;

// ESP-Now
esp_now_peer_info_t peerInfo;

// PINMODE
void pinSetup() {
  Serial.begin(115200);
  pinMode(FLY_LEFT_DIR, OUTPUT);
  pinMode(FLY_RIGHT_DIR, OUTPUT);
  pinMode(FLY_LEFT_PWM, OUTPUT);
  pinMode(FLY_RIGHT_PWM, OUTPUT);
  pinMode(LIN_ACT_DIR, OUTPUT);
  pinMode(LIN_ACT_PWM, OUTPUT);
  pinMode(TURN_DIR, OUTPUT);
  pinMode(TURN_PWM, OUTPUT);
  pinMode(RELOAD_DIR, OUTPUT);
  pinMode(RELOAD_PWM, OUTPUT);
  pinMode(SHOOT_DIR, OUTPUT);
  pinMode(SHOOT_PWM, OUTPUT);
  pinMode(SHOOT_SW, INPUT_PULLUP);
  pinMode(RELOAD_SW1, INPUT_PULLUP);
  pinMode(RELOAD_SW2, INPUT_PULLUP);
  pinMode(TUR_POT, INPUT);
}

void initMotors() {
  turntable.setMaxSpeed(1000);
  turntable.setAcceleration(1500);
  
  // Moves Shooting Mechanism down until switch is triggered
  Serial.println("Calibrating Shooting");
  shootMotor.setMaxSpeed(12000);
  shootMotor.move(-20000);
  shootMotor.setAcceleration(1500);
  while (digitalRead(SHOOT_SW) == HIGH) {
    shootMotor.run();
  }
  shootMotor.setCurrentPosition(0);
  
  // Moves Reloading Mechanism up until Switch is triggered
  Serial.println("Calibrating Reloading");
  reloadMotor.setMaxSpeed(3000);
  reloadMotor.move(-20000);
  reloadMotor.setAcceleration(1500);
  while (digitalRead(RELOAD_SW1) == HIGH) {
    Serial.print(".");
    reloadMotor.run();
  }
  Serial.println("TOUCHED");
  reloadMotor.setCurrentPosition(0);
  reloadMotor.setAcceleration(750);
}

void triggerShoot() {
  // Actuator
  shootMotor.setAcceleration(6500);
  shootMotor.moveTo(2055);
  while (shootMotor.distanceToGo() != 0) {
    shootMotor.run();
  }
  shootMotor.move(-20000);
  while (digitalRead(SHOOT_SW) == HIGH) {
    shootMotor.run();
  }
  shootMotor.setCurrentPosition(0);
}

void triggerReload(float state) {
  if (state) {
    reloadMotor.move(20000);
    if (digitalRead(RELOAD_SW2) == HIGH) {
      reloadMotor.run();
    } else {
      reloadMotor.stop();
      reloadMotor.setCurrentPosition(0);
    }
  } else {
    reloadMotor.move(-200000);
    if (digitalRead(RELOAD_SW1) == HIGH) {
      reloadMotor.run();
    } else {
      reloadMotor.stop();
      reloadMotor.setCurrentPosition(0);
    }
  }
}

void updateFlywheel() {
  if (robCon.flywheel > 0) {
    digitalWrite(FLY_LEFT_DIR, HIGH);
    ledcWrite(0, robCon.flywheel);
    digitalWrite(FLY_RIGHT_DIR, LOW);
    ledcWrite(1, robCon.flywheel);
  } else if (robCon.flywheel <= 0) {
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}

void updateTurntable() {
  if (int(robCon.turnTbl) % 2 == 0) {
    turntable.moveTo(robCon.turnTbl  * 16 * 5 / 1.8); // Requested Pos (angle) * MicroStep * Gear Ratio / angle p step
  }
  turntable.run();
}

void updateLinearAct() {
  cubeAngle = 0.000015 * pow(robCon.turretAng, 3);
  if (robCon.turretAng > 0) {
    digitalWrite(LIN_ACT_DIR, LOW);
    ledcWrite(2, cubeAngle);
  } else {
    digitalWrite(LIN_ACT_DIR, HIGH);
    ledcWrite(2, -1 * cubeAngle);
  }
}

void updatePneumatics() {
  if (robCon.ring_1) {
    MCP.digitalWrite(0, LOW);
  } else {
    MCP.digitalWrite(0, HIGH);
  }
  
  if (robCon.ring_2) {
    MCP.digitalWrite(1, LOW);
    delay(50);
    MCP.digitalWrite(1, HIGH);
  }
}

void updateTurAng() {
  potAng.pot = analogRead(TUR_POT);
  esp_now_send(broadcastAdd, (uint8_t *) &potAng, sizeof(potAng));
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&robCon, incomingData, sizeof(robCon));
}

void setup() {
  pinSetup();
  initMotors();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error Initializing ESP-NOW");
    return;
  }

  // Register ESP-NOW peer
  memcpy(peerInfo.peer_addr, broadcastAdd, 6);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(3, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Register receiver function
  esp_now_register_recv_cb(onDataRecv);

  // Set Board Pin IN/OUT
  pinSetup();

  // Setup PWM
  int num_PWM = 3;
  for (int i = 0; i < num_PWM; i++) {
    ledcSetup(i, 10000, 8);
  }
  ledcAttachPin(FLY_LEFT_PWM, 0);
  ledcAttachPin(FLY_RIGHT_PWM, 1);
  ledcAttachPin(LIN_ACT_PWM, 2);

  if (!MCP.begin_I2C()) {
    Serial.println("MCP I2C Error");
    return;
  }

  MCP.pinMode(0, OUTPUT);
  MCP.pinMode(1, OUTPUT);
  MCP.digitalWrite(0, HIGH);
  MCP.digitalWrite(1, HIGH);
  
  turntable.moveTo(robCon.turnTbl  * 16 * 5 / 1.8);
}



void loop() {
  // Report turret angle to controller
  updateTurAng();

  // Ring System Control
  updatePneumatics();

  // Flywheel Control
  updateFlywheel();

  // Linear Actuator Control
  updateLinearAct();

  updateTurntable();

  // SHOOT
  if (robCon.shootTrg) {
    triggerShoot();
  }

  // RELOAD
  triggerReload(robCon.reloadTrg);
}