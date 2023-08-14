#include <esp_now.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include "esp_private/wifi.h"

#define x_pin 35
#define y_pin 34
#define z_pin 33
#define d_pin 32
#define SW1 21
#define SW2 22
#define B1 4 //Triangle
#define B2 13 //Square
#define B3 27 //Circle
#define B4 26  // X
#define L1 17 //L1  Reload
#define R1 16 //R1  Shoot
#define pot1 36
#define pot2 39
#define led_calibration 2
#define ESP_NOW_sent 15
#define DEG2RAD 0.0174532925

const int MaxReadings = 50; //For smoothening joystickj; Adjustable; larger number = more delay but smoother.
const int motor_max_rpm = 250;   //Max rpm for robot's motor
const int motor_min_rpm = -motor_max_rpm; //Set minimum value
const double wheel_radius = 0.076;   //meters
const double wheel_separation = 0.6; //meters, (distance between center of left and right, front and back wheels)
const int ADC_bit = 12;
float x, y, z, d; 
int joystickx, joysticky, joystickz, joystickd, readpot1, readpot2; 
float FLW, FRW, RLW, RRW; //Wheel speeds
bool ledState = false;
bool refreshTenthPOT1 = false;
bool refreshHundredthPOT1 = false;
bool refreshTenthPOT2 = false;
bool refreshHundredthPOT2 = false;
bool refreshTenthPitch = false;
bool refreshHundredthPitch = false;
bool refreshThousandthPitch = false;
bool ten = false;
bool hdrd = false;

int xcenter, ycenter, zcenter, dcenter; 
int xMin = 0, xMax = 4095;
int yMin = 0, yMax = 4095;
int zMin = 0, zMax = 4095;
int dMin = 0, dMax = 4095;
int pot1Min = 0 , pot1Max = 4095;
int pot2Min = 0 , pot2Max = 4095;

int Xreadings[MaxReadings];
int XreadIndex = 0;
int Xtotal = 0;
int X_Pos = 0;

int Yreadings[MaxReadings];
int YreadIndex = 0;
int Ytotal = 0;
int Y_Pos = 0;

int Zreadings[MaxReadings];
int ZreadIndex = 0;
int Ztotal = 0;
int Z_Pos = 0;

int Dreadings[MaxReadings];
int DreadIndex = 0;
int Dtotal = 0;
int D_Pos = 0;

int Pot1readings[MaxReadings];
int Pot1readIndex = 0;
int Pot1total = 0;
int Pot1_Pos = 0;

int Pot2readings[MaxReadings];
int Pot2readIndex = 0;
int Pot2total = 0;
int Pot2_Pos = 0;

// TFT Config
TFT_eSPI tft = TFT_eSPI();

//ESP-now config
uint8_t broadcastAddress1[] = {0x30, 0xAE, 0xA4, 0x99, 0xA3, 0xD8}; //Base
uint8_t broadcastAddress2[] = {0x40, 0x22, 0xD8, 0x77, 0x7D, 0x3C}; //Turret

typedef struct rabbit_robot{
  float FLW, FRW, RLW, RRW; // Base
  float angle, flywheel_speed, lazy_susan_rotate, reload, shoot; //Turret
  bool shootSequence, ring_1, ring_2;
} rabbit_robot;

typedef struct pot_ang {
  int pot;
} pot_ang;

pot_ang potAng;
rabbit_robot base;
rabbit_robot turret;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
 digitalWrite(ESP_NOW_sent, HIGH);
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&potAng, incomingData, sizeof(potAng));
}

void calibrate() {
  Serial.println("\n---calibrating joystick---\n");
  Serial.println("place the joystick in the center position");
  delay(1000);
  joystickx= joysticky= joystickz = joystickd= readpot1 = readpot2 = 0;
  digitalWrite(led_calibration, HIGH);
  for (int i = 0; i < 100; i++) {
    Serial.print(".");
    joystickx += analogRead(x_pin);
    delay(5);
    joysticky += analogRead(y_pin);
    delay(5);
    joystickz += analogRead(z_pin);
    delay(5);
    joystickd += analogRead(d_pin);
    delay(5);
    readpot1 += analogRead(pot1);
    delay(5);
    readpot2 += analogRead(pot2);
  }
  xcenter = joystickx/100;
  ycenter = joysticky/100; 
  zcenter = joystickz/100;
  dcenter = joystickd/100;
  delay (3000);
  digitalWrite(led_calibration, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(led_calibration, OUTPUT);
  pinMode(ESP_NOW_sent, OUTPUT);
  pinMode(B1, INPUT);
  pinMode(B2, INPUT);
  pinMode(B3, INPUT);
  pinMode(B4, INPUT);

  //Initialise ESP-NOW
  WiFi.mode(WIFI_STA);
  if(esp_now_init() != ESP_OK){
    Serial.println("Error");
    return;
  }

  //Register callback
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(onDataRecv);

  //Register Peer
  esp_now_peer_info_t peerInfo;
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(3, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  //Add peer
  memcpy (peerInfo.peer_addr, broadcastAddress1, 6);
  if(esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  //Add peer
  memcpy (peerInfo.peer_addr, broadcastAddress2, 6);
  if(esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  calibrate();
}

void loop() {
  //filter for joysticks
  Xtotal -= Xreadings[XreadIndex];
  Ytotal -= Yreadings[YreadIndex];
  Ztotal -= Zreadings[ZreadIndex];
  Dtotal -= Dreadings[DreadIndex];
  Pot1total -= Pot1readings[Pot1readIndex];
  Pot2total -= Pot2readings[Pot2readIndex];
  Xreadings[XreadIndex] = analogRead(x_pin);
  Yreadings[YreadIndex] = analogRead(y_pin);
  Zreadings[ZreadIndex] = analogRead(z_pin);
  Dreadings[DreadIndex] = analogRead(d_pin);
  Pot1readings[Pot1readIndex] = analogRead(pot1);
  Pot2readings[Pot2readIndex] = analogRead(pot2);
  Xtotal = Xtotal + Xreadings[XreadIndex];
  Ytotal = Ytotal + Yreadings[YreadIndex];
  Ztotal = Ztotal + Zreadings[ZreadIndex];
  Dtotal = Dtotal + Dreadings[DreadIndex];
  Pot1total = Pot1total + Pot1readings[Pot1readIndex];
  Pot2total = Pot2total + Pot2readings[Pot2readIndex];
  XreadIndex = XreadIndex + 1;
  YreadIndex = YreadIndex + 1;
  ZreadIndex = ZreadIndex + 1;
  DreadIndex = DreadIndex + 1;
  Pot1readIndex = Pot1readIndex + 1;
  Pot2readIndex = Pot2readIndex + 1;
  if (XreadIndex >= MaxReadings) XreadIndex = 0;
  if (YreadIndex >= MaxReadings) YreadIndex = 0;
  if (ZreadIndex >= MaxReadings) ZreadIndex = 0;
  if (DreadIndex >= MaxReadings) DreadIndex = 0;
  if (Pot1readIndex >= MaxReadings) Pot1readIndex = 0;
  if (Pot2readIndex >= MaxReadings) Pot2readIndex = 0;
  X_Pos = Xtotal / MaxReadings;
  Y_Pos = Ytotal / MaxReadings;
  Z_Pos = Ztotal / MaxReadings;
  D_Pos = Dtotal / MaxReadings;
  Pot1_Pos = Pot1total / MaxReadings;
  Pot2_Pos = Pot2total / MaxReadings;
  if (X_Pos > xcenter + 100) {
    x = map(X_Pos, xcenter , xMax, 0, 1000);
    x = x / 1000;
  } else if (X_Pos < xcenter - 100) {
    x = map(X_Pos, xcenter , xMin, 0, 1000);
    x = x / -1000;
  } else{
    x = 0;
  }
  if (Y_Pos > ycenter + 100) {
    y = map(Y_Pos, ycenter , yMax, 0, 1000);
    y = y / -1000;
  } else if (Y_Pos < ycenter - 100){
    y = map(Y_Pos, ycenter  , yMin, 0, 1000);
    y = y / 1000;
  } else{
    y = 0;
  }
    if (Z_Pos > zcenter + 100) {  // Right Joy Stick X-axis
    z = map(Z_Pos, zcenter , zMax, 0, 1000);
    z = z / -1000;
  } else if (Z_Pos < zcenter - 100){
    z = map(Z_Pos, zcenter  , zMin, 0, 1000);
    z = z / 1000;
  } else{
    z = 0;
  }
    if (D_Pos > dcenter + 500) {  // Right Joy Stick Y-axis
    d = map(D_Pos, dcenter , dMax, 0, -255);
  } else if (D_Pos < dcenter - 500){
    d = map(D_Pos, dcenter  , dMin, 0, 255);
  } else{
    d = 0;
  }
  
  //Inverse Kinematics
  FLW = constrain((y + x - wheel_separation * z) * motor_max_rpm, motor_min_rpm, motor_max_rpm);
  FRW = constrain((y - x + wheel_separation * z) * motor_max_rpm, motor_min_rpm, motor_max_rpm);
  RLW = constrain((y - x - wheel_separation * z) * motor_max_rpm, motor_min_rpm, motor_max_rpm);
  RRW = constrain((y + x + wheel_separation * z) * motor_max_rpm, motor_min_rpm, motor_max_rpm);

  Pot1_Pos = map(Pot1_Pos, 0, 4095, -150, 150);
  Pot2_Pos = map(Pot2_Pos, 0, 4095, 0, 200);

  // Special Function for Square
  //Send data to Base via ESP-NOW
  base.FLW = FLW;
  base.FRW = FRW;
  base.RLW = RLW;
  base.RRW = RRW;
  turret.angle = d;
  turret.lazy_susan_rotate = Pot1_Pos;
  turret.flywheel_speed = Pot2_Pos;
  turret.shootSequence = digitalRead(B1);
  if (digitalRead(B2) == HIGH) {
    turret.ring_1 = true;
  } else if (digitalRead(B4) == HIGH) {
    turret.ring_1 = false;
  }
  turret.ring_2 = digitalRead(B3);
  turret.shoot = digitalRead(R1);
  turret.reload = digitalRead(L1);
  
  esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t *) &base, sizeof(base));
  esp_err_t result2 = esp_now_send(broadcastAddress2, (uint8_t *) &turret, sizeof(turret));

  ledState = !ledState;
  digitalWrite(15, ledState);
  if (digitalRead(SW1) == LOW) {
    displayStuff();
  }
}

void displayStuff() {
  if (Pot1_Pos < 10 && !refreshTenthPOT1) {
    refreshTenthPOT1 = true;
    tft.fillScreen(TFT_BLACK);
  } else if (Pot1_Pos > 10) {
    refreshTenthPOT1 = false;
  }
  if (Pot1_Pos < 100 && !refreshHundredthPOT1) {
    refreshHundredthPOT1 = true;
    tft.fillScreen(TFT_BLACK);
  } else if (Pot1_Pos > 100) {
    refreshHundredthPOT1 = false;
  }
  if (Pot2_Pos < 10 && !refreshTenthPOT2) {
    refreshTenthPOT2 = true;
    tft.fillScreen(TFT_BLACK);
  } else if (Pot2_Pos > 10) {
    refreshTenthPOT2 = false;
  }
  if (Pot2_Pos < 100 && !refreshHundredthPOT2) {
    refreshHundredthPOT2 = true;
    tft.fillScreen(TFT_BLACK);
  } else if (Pot2_Pos > 100) {
    refreshHundredthPOT2 = false;
  }
  if (potAng.pot < 10 && !refreshTenthPitch) {
    refreshTenthPitch = true;
    tft.fillScreen(TFT_BLACK);
  } else if (potAng.pot > 10) {
    refreshTenthPitch = false;
  }
  if (potAng.pot < 100 && !refreshHundredthPitch) {
    refreshHundredthPitch = true;
    tft.fillScreen(TFT_BLACK);
  } else if (potAng.pot > 100) {
    refreshHundredthPitch = false;
  }
  if (potAng.pot < 1000 && !refreshThousandthPitch) {
    refreshThousandthPitch = true;
    tft.fillScreen(TFT_BLACK);
  } else if (potAng.pot > 1000) {
    refreshThousandthPitch = false;
  }

  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.print("Yaw: ");
  tft.setTextSize(5);
  if (Pot1_Pos == 0) {
    tft.print(" ");
  }
  if (Pot1_Pos > 0) {
    tft.print("+");
  }
  tft.println(Pot1_Pos);


  tft.setCursor(10, 100);
  tft.setTextSize(2);
  tft.print("Flywheel Speed: ");
  tft.setTextSize(5);
  tft.println(Pot2_Pos);

  tft.setCursor(10, 200);
  tft.setTextSize(2);
  tft.print("Pitch: ");
  tft.setTextSize(5);
  int pot_a = map(potAng.pot, 2930, 1999, 0, 100);
  if (pot_a >= 10) {
    ten = true;
  }
  if (pot_a >= 100) {
    hdrd = true;
  }
  if (pot_a <= 0) {
    pot_a = 0;
  }
  if (pot_a < 10 && ten) {
    tft.fillRect(120, 200, 400, 250, TFT_BLACK);
    ten = false;
  }
  if (pot_a < 100 && hdrd) {
    tft.fillRect(150, 200, 300, 250, TFT_BLACK);
    hdrd = false;
  }
  tft.println(pot_a);
}