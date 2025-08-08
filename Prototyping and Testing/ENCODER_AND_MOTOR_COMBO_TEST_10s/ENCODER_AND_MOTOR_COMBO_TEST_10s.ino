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
const unsigned long printInterval = 100; // 100 ms
const unsigned long runDuration = 10000; // 10 seconds
unsigned long startTime = 0;

// Rotation tracking
float previousAngle = 0;
float totalAngleUnwrapped = 0;
int totalRotations = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  magenc_LF.begin();

  pinMode(PWM1, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(SPD_CTRL_A, INPUT);

  motorForward();

  Serial.println("time_ms,speed_pwm,angle_raw,angle_deg,status,angle_unwrapped");

  startTime = millis();
  previousAngle = magenc_LF.rawAngle() * 360.0 / 4096.0; // Initial angle in degrees
}

void loop() {
  unsigned long now = millis();

  // Stop after 10 seconds
  if (now - startTime >= runDuration) {
    analogWrite(PWM1, 0);
    motorBrake();

    float totalTimeSec = (now - startTime) / 1000.0;
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

    while (1); // Halt
  }

  if (now - lastPrintTime >= printInterval) {
    lastPrintTime = now;

    int speedPWM = map(analogRead(SPD_CTRL_A), 0, 1023, 0, 255);
    analogWrite(PWM1, speedPWM);

    uint16_t angleRaw = magenc_LF.rawAngle();
    float currentAngle = angleRaw * 360.0 / 4096.0;
    uint8_t status = magenc_LF.readStatus();

    // Angle unwrapping with direction correction
    float delta = currentAngle - previousAngle;

    if (delta < -180.0) {
      delta += 360.0;
    } else if (delta > 180.0) {
      delta -= 360.0;
    }

    delta *= -1; // Correct for reverse-sensing encoder

    totalAngleUnwrapped += delta;
    totalRotations = totalAngleUnwrapped / 360.0;
    previousAngle = currentAngle;

    // CSV output
    Serial.print(now - startTime); Serial.print(",");
    Serial.print(speedPWM); Serial.print(",");
    Serial.print(angleRaw); Serial.print(",");
    Serial.print(currentAngle, 2); Serial.print(",");
    Serial.print(status, BIN); Serial.print(",");
    Serial.println(totalAngleUnwrapped, 2);
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
