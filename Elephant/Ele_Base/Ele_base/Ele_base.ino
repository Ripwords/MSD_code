/*

Rabbit Robot Base Code

Last Updated: 7th June 2023

*/

//Libraries
#include <esp_now.h>
#include "esp_private/wifi.h"
#include <WiFi.h>
#include <Wire.h>

// Robot base motors movement parameters
// Configure based on robot base and microcontroller
const int motor_max_rpm = 250;             //rpm
const int motor_min_rpm = -motor_max_rpm;  //Set minimum value
const int Number_of_motors = 4;
const double wheel_radius = 0.076;     //meters
const double wheel_separation = 0.6;  //meters, (distance between center of left and right, front and back wheels)

int chan = 3;
int target[Number_of_motors];
int ENCA[Number_of_motors] = { 13, 23, 34, 26 };
int ENCB[Number_of_motors] = { 19, 32, 25, 35 };
int PWM[Number_of_motors] = { 5, 2, 4, 16 };
int DIR[Number_of_motors] = { 18, 33, 17, 14 };
volatile int counter[Number_of_motors] = { 0, 0, 0, 0 };
unsigned long time_elapsed[Number_of_motors] = { 0, 0, 0, 0 };
unsigned long last_time[Number_of_motors] = { 0, 0, 0, 0 };
unsigned long current_time[Number_of_motors];
float rpm[Number_of_motors];
float output[Number_of_motors] = { 0, 0, 0, 0 };
float error[Number_of_motors] = { 0, 0, 0, 0 };
float last_error[Number_of_motors] = { 0, 0, 0, 0 };
float prev_error[Number_of_motors] = { 0, 0, 0, 0 };
float proportional[Number_of_motors] = { 0, 0, 0, 0 };
float integral[Number_of_motors] = { 0, 0, 0, 0 };
float derivative[Number_of_motors] = { 0, 0, 0, 0 };
float Kp[Number_of_motors] = { 0, 0, 0, 0 };
float Ki[Number_of_motors] = { 5, 5, 5, 5 }; // changed from 4 to 5
float Kd[Number_of_motors] = { 0, 0, 0, 0 };


// ESP-NOW Receiver's MAC address
uint8_t broadcastAddress[] = { 0x0C, 0xB8, 0x15, 0xF8, 0xA1, 0x24 };

// Define received data structure
typedef struct rabbit_robot {
  float FLW, FRW, RLW, RRW;                                       // Base
  float angle, flywheel_speed, lazy_susan_rotate, reload, shoot, reload_rotation;  //Turret
  bool emergencyReloadStop;
  bool enable_flywheel;
  bool dropRing;
  bool shootSequence;

} rabbit_robot;

// Declare structure object
rabbit_robot base;

esp_now_peer_info_t peerInfo;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&base, incomingData, sizeof(base));
  target[0] = base.FLW;
  target[1] = base.FRW;
  target[2] = base.RLW;
  target[3] = base.RRW;
}

// encoder interrupt functions
void IRAM_ATTR read_encoder1() {
  //Serial.println("ENCA Triggered on encoder 1");
  int a = digitalRead(ENCB[0]);
  if (a == HIGH) {
    counter[0]++;
  } else {
    counter[0]--;
  }
}

void IRAM_ATTR read_encoder2() {
  //Serial.println("ENCA Triggered on encoder 2");
  int b = digitalRead(ENCB[1]);
  if (b == HIGH) {
    counter[1]++;
  } else {
    counter[1]--;
  }
}

void IRAM_ATTR read_encoder3() {
  //Serial.println("ENCB Triggered on encoder 3");
  int c = digitalRead(ENCB[2]);
  if (c == HIGH) {
    counter[2]++;
  } else {
    counter[2]--;
  }
}

void IRAM_ATTR read_encoder4() {
  //Serial.println("ENCB Triggered on encoder 4");
  int d = digitalRead(ENCB[3]);
  if (d == HIGH) {
    counter[3]++;
  } else {
    counter[3]--;
  }
}

void setup() {
  Serial.begin(115200);
  //Initialise ESP-Now
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register ESP-NOW peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(chan, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  // setup encoders and motors code
  for (int i = 0; i < Number_of_motors; i++) {
    pinMode(ENCA[i], INPUT_PULLUP);
    pinMode(ENCB[i], INPUT_PULLUP);
    pinMode(PWM[i], OUTPUT);
    pinMode(DIR[i], OUTPUT);

    ledcSetup(i, 10000, 8);
    ledcAttachPin(PWM[i], i);

/*
    attachInterrupt(digitalPinToInterrupt(ENCA[0]), read_encoder1, RISING);  // attach interrupt to ENCA pin
    attachInterrupt(digitalPinToInterrupt(ENCA[1]), read_encoder2, RISING);  // attach interrupt to ENCA pin
    attachInterrupt(digitalPinToInterrupt(ENCA[2]), read_encoder3, RISING);  // attach interrupt to ENCA pin
    attachInterrupt(digitalPinToInterrupt(ENCA[3]), read_encoder4, RISING);  // attach interrupt to ENCA pin
*/
    attachInterrupt(ENCA[0], read_encoder1, RISING);  // attach interrupt to ENCA pin
    attachInterrupt(ENCA[1], read_encoder2, RISING);  // attach interrupt to ENCA pin
    attachInterrupt(ENCA[2], read_encoder3, RISING);  // attach interrupt to ENCA pin
    attachInterrupt(ENCA[3], read_encoder4, RISING);  // attach interrupt to ENCA pin

    last_time[0] = micros();
    last_time[1] = micros();
    last_time[2] = micros();
    last_time[3] = micros();
  }
}

void loop() {
  // Print all 4 motors' expected speed (from the controller)
  Serial.print(target[0]);
  Serial.print(" ");
  Serial.print(target[1]);
  Serial.print(" ");
  Serial.print(target[2]);
  Serial.print(" ");
  Serial.println(target[3]);

  // PID control to achieve predictable and smooth control of the robot movement
  for (int i = 0; i < Number_of_motors; i++) {
    current_time[i] = micros();
    
    if (current_time[i] - last_time[i] >= 10000) {  // calculate time elapsed since last loop (in microseconds)
      noInterrupts();                               // disable interrupts to read counter variable
      time_elapsed[i] = current_time[i] - last_time[i];
      rpm[i] = (counter[i] * 4 / 120.0) / (time_elapsed[i] / 1000000.0) * 60.0;  // calculate RPM based on time elapsed and counter variable
      rpm[i] = constrain(rpm[i], motor_min_rpm, motor_max_rpm);
      counter[i] = 0;                  // reset counter variable
      last_time[i] = current_time[i];  // update last time variable
      interrupts();                    // re-enable interrupts

      // RPM control
      error[i] = target[i] - rpm[i];
      proportional[i] = Kp[i] * (error[i] - last_error[i]);
      integral[i] += Ki[i] * error[i] * (time_elapsed[i] / 1000000.0);
      derivative[i] = Kd[i] * (error[i] - last_error[i]) / (time_elapsed[i] / 1000000.0);
      output[i] = proportional[i] + integral[i] + derivative[i];
      output[i] = constrain(map(output[i], motor_min_rpm, motor_max_rpm, -255, 255), -255, 255);

      // Output the calculated speed values to each motor
      // Direction output
      if (output[0] < 0) {
        digitalWrite(DIR[0], HIGH);
      } else if (output[0] >= 0) {
        digitalWrite(DIR[0], LOW);
      }

      if (output[1] < 0) {
        digitalWrite(DIR[1], LOW);
      } else if (output[1] >= 0) {
        digitalWrite(DIR[1], HIGH);
      }

      if (output[2] < 0) {
        digitalWrite(DIR[2], HIGH);
      } else if (output[2] >= 0) {
        digitalWrite(DIR[2], LOW);
      }

      if (output[3] < 0) {
        digitalWrite(DIR[3], LOW);
      } else if (output[3] >= 0) {
        digitalWrite(DIR[3], HIGH);
      }
      
      // Speed output
      ledcWrite(i, abs(output[i]));

      prev_error[i] = last_error[i];
      last_error[i] = error[i];
    }
  }
}
