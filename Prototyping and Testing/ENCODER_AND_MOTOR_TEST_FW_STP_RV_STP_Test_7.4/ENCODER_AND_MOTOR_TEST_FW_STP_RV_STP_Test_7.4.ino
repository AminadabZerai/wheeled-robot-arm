/* TB6612FNG DUAL MOTOR DRIVER AND AS5600 MAGNETIC ENCODER TEST

 * TEST SETUP - Arduino Nano 33 IoT 3.3V Logic, Motor Driven at 7.4V
 * Motor runs FORWARD for 7 seconds at maximum PWM(255), STOPS for 3 seconds,
 * runs in REVERSE for 7 seconds, then brakes.
 * Encoder angle unwrapping is reset to zero at every code reset.
 * Aminadab Z. Gherbehiwet - 30/7/2025
 */

#include <Wire.h>
#include "AS5600.h"

// Pin definitions
#define PWM1 5
#define AIN1 3
#define AIN2 4
#define SPD_CTRL_A A0

AS5600 magenc_LF;

// Timing
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 100;
unsigned long startTime = 0;

const unsigned long FORWARD_TIME_1 = 7000;
const unsigned long STOP1_TIME = FORWARD_TIME_1 + 3000;
const unsigned long REVERSE_TIME_1 = STOP1_TIME + 7000;
const unsigned long STOP2_TIME = REVERSE_TIME_1 + 3000;

const unsigned long REVERSE_TIME_2 = STOP2_TIME + 7000;
const unsigned long STOP3_TIME = REVERSE_TIME_2 + 3000;
const unsigned long FORWARD_TIME_2 = STOP3_TIME + 7000;
const unsigned long STOP4_TIME = FORWARD_TIME_2 + 1000;

// Rotation tracking
float previousAngle = 0;
float totalAngleUnwrapped = 0;
int totalRotations = 0;
float angleZeroOffset = 0.0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  magenc_LF.begin();

  pinMode(PWM1, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(SPD_CTRL_A, INPUT);

  // Use current angle as zero
  angleZeroOffset = magenc_LF.rawAngle() * 360.0 / 4096.0;
  previousAngle = 0;

  Serial.println("time_ms,speed_pwm,angle_raw,angle_deg,status,angle_unwrapped,motorstatus");

  delay(3000);
  startTime = millis();
}

void loop() {
  unsigned long now = millis();
  unsigned long elapsed = now - startTime;

  // Motor phase control
  String motorState;
  if (elapsed <= FORWARD_TIME_1) {
    motorForward();
    motorState = "FORWARD_1";
  } else if (elapsed <= STOP1_TIME) {
    motorBrake();
    motorState = "STOP1";
  } else if (elapsed <= REVERSE_TIME_1) {
    motorReverse();
    motorState = "REVERSE_1";
  } else if (elapsed <= STOP2_TIME) {
    motorBrake();
    motorState = "STOP2";
  } else if (elapsed <= REVERSE_TIME_2) {
    motorReverse();
    motorState = "REVERSE_2";
  } else if (elapsed <= STOP3_TIME) {
    motorBrake();
    motorState = "STOP3";
  } else if (elapsed <= FORWARD_TIME_2) {
    motorForward();
    motorState = "FORWARD_2";
  } else if (elapsed <= STOP4_TIME) {
    motorBrake();
    motorState = "STOP4";
  } else {
    // Final stop & summary
    analogWrite(PWM1, 0);
    motorBrake();

    float totalTimeSec = (elapsed) / 1000.0;
    float angularSpeedRadPerSec = (totalAngleUnwrapped * DEG_TO_RAD) / totalTimeSec;

    Serial.println();
    Serial.print("Total unwrapped angle: ");
    Serial.print(totalAngleUnwrapped, 2);
    Serial.println(" degrees");

    Serial.print("Total rotations: ");
    Serial.println(totalRotations);

    Serial.print("Average angular velocity: ");
    Serial.print(angularSpeedRadPerSec, 4);
    Serial.println(" rad/s");

    while (1); // Stop execution
  }

  if (now - lastPrintTime >= printInterval) {
    lastPrintTime = now;

    // Hardcoded full speed
    int speedPWM = 255;
    analogWrite(PWM1, speedPWM);

    uint16_t angleRaw = magenc_LF.rawAngle();
    float currentAngle = (angleRaw * 360.0 / 4096.0) - angleZeroOffset;

    if (currentAngle < 0) currentAngle += 360.0;
    if (currentAngle >= 360.0) currentAngle -= 360.0;

    uint8_t status = magenc_LF.readStatus();

    // Angle unwrapping logic (bidirectional)
    float delta = currentAngle - previousAngle;
    if (delta < -180.0) {
      delta += 360.0;
    } else if (delta > 180.0) {
      delta -= 360.0;
    }

    totalAngleUnwrapped += delta;
    totalRotations = totalAngleUnwrapped / 360.0;
    previousAngle = currentAngle;

    // CSV log
    Serial.print(elapsed); Serial.print(",");
    Serial.print(speedPWM); Serial.print(",");
    Serial.print(angleRaw); Serial.print(",");
    Serial.print(currentAngle, 2); Serial.print(",");
    Serial.print(status, BIN); Serial.print(",");
    Serial.print(-1*totalAngleUnwrapped, 2); Serial.print(",");
    Serial.println(motorState);
  }
}

// Motor control functions
void motorForward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}

void motorReverse() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
}

void motorBrake() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}
