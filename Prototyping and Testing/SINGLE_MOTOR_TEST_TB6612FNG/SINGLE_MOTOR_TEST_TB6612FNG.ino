/* TB6612FNG DUAL MOTOR DRIVER AND AS5600 MAGNETIC ENCODER TEST
 * Aminadab Z. Gherbehiwet - 18/7/2025
 */

/* GUIDE TABLE FOR MOTOR CONTROL
        AIN1/BIN1     AIN2/BIN2
--------------------------------------------
FWR     HIGH             LOW
REV     LOW              HIGH
BRK     LOW              LOW
BRK     HIGH             HIGH
 */


#include <Wire.h>
#include "AS5600.h"


// MOTOR A - TB6612FNG CONNECTIONS
#define PWM1 5
#define AIN1 3
#define AIN2 4

// AS5600 Object Creation
AS5600 magenc_LF;



int SPD_CTRL_A = A0;  // Pin to read Potentiometer Value

// For printing speed and encoder data
unsigned long lastPrintTime = 0;
unsigned long printInterval = 500; // ms


// For controlling motor states
unsigned long stateStartTime = 0;
enum MotorState { FORWARD, STOP_AFTER_FWD};
MotorState currentState = FORWARD;


void setup() {
  Serial.begin(115200);

  while(!Serial);
  Wire.begin();
  magenc_LF.begin();


  pinMode(PWM1, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  if (magenc_LF.isConnected()) {
    Serial.println("LEFT_FWD_MAG_ENC connected!");
  } else {
    Serial.println("LEFT_FWD_MAG_ENC not found.");
  }

  stateStartTime = millis(); // Start the Finite State Machine
}

void loop() {
  unsigned long now = millis();
  uint8_t status = magenc_LF.readStatus();

  // Read the status of the encoder

  // Read potentiometer and set speed
  int motorSpeedA = map(analogRead(SPD_CTRL_A), 0, 1023, 0, 255);
  analogWrite(PWM1, motorSpeedA);

  // Print Speed and Encoder data every 500 ms
  if (now - lastPrintTime >= printInterval) {
    lastPrintTime = now;
    Serial.print("Speed: ");
    Serial.print(motorSpeedA);
    Serial.print("    State: ");
    switch (currentState) {
      case FORWARD: Serial.println("FORWARD"); break;
      case STOP_AFTER_FWD: Serial.println("STOP AFTER FORWARD"); break;
    }

    Serial.print("Status: ");
    Serial.print(status, BIN);
    Serial.print("  Angle: ");
    Serial.print(magenc_LF.readAngle());
    Serial.print("  Raw: ");
    Serial.println(magenc_LF.rawAngle());
  }

  // FSM logic (non-blocking)
  switch (currentState) {
    case FORWARD:
      motorForward();
      if (now - stateStartTime >= 5000) { // 5s
        currentState = STOP_AFTER_FWD;
        stateStartTime = now;
        motorBrake();
      }
      break;

    case STOP_AFTER_FWD:
      if (now - stateStartTime >= 2000) { // 2s
        currentState = FORWARD;
        stateStartTime = now;
      }
      break;
  }
}

void motorForward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}

void motorBrake() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

