/* TB6612FNG DUAL MOTOR DRIVER AND AS5600 MAGNETIC ENCODER PID CONTROL TEST

 * TEST SETUP - Arduino Nano 33 IoT 3.3V Logic, Motor Driven at 7.4V
 * Direction and desired postition will be given to the motor throuhg serial input and
 * a PID Controller with tunable paramters Kp,Ki,Kd will be implemented to see how well the motor
 * responds to a given set-value.
 * Aminadab Z. Gherbehiwet - 5/8/2025
 */

#include <Wire.h>
#include "AS5600.h"

// Pin definitions
#define PWM1 5
#define AIN1 3
#define AIN2 4

AS5600 encoder_RF;    // Creating Encoder Object

// Timing
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500;

unsigned long lastControlTime = 0;
const unsigned long controlInterval = 20;

unsigned long startTime = 0;

// Rotation tracking
float previousAngle = 0;
float totalAngleUnwrapped = 0;
int totalRotations = 0;

// PID Variables
float setpoint = 0;
float measured = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float last_Error = 0;
float dt = 0.0;
int pwm;
char dir;
float delta = 0.0;

// PID Tuning Parameters
float kp = 0.6;
float ki = 0.3;
float kd = 0.001;

// Motor Setup Function Prototype
void setMotor(char dir, int pwmVal);

void setup() {

  // Initializations
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  encoder_RF.begin();

  pinMode(PWM1, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  // Zero the encoder
  previousAngle = 0.0;

  startTime = millis();

  Serial.println("Enter direction and setpoint (deg/s), e.g., f,150");
}

void loop() {
  unsigned long now = millis();

  // Read and parse user command
  if (Serial.available() > 0) {
    String user_cmd = Serial.readStringUntil('\n'); // read full line
    user_cmd.trim();                                // remove whitespaces and newline

    int comma_idx = user_cmd.indexOf(',');          // Locate comma
    if (comma_idx > 0 && user_cmd.length() > comma_idx + 1) {
      char direction = user_cmd.charAt(0); // Get the first character for direction
      float speed = user_cmd.substring(comma_idx + 1).toFloat();   // Get the numerical part after comma
      setpoint = (direction == 'r' || direction == 'R') ? -abs(speed) : abs(speed);
      Serial.print("Received Setpoint: ");
      Serial.print(setpoint);
      Serial.println(" deg/s");
    } else {
      Serial.println("Invalid format. Use f,100 or r,90");
    }
  }

  if (now - lastControlTime >= controlInterval) {
    dt = (controlInterval) / 1000.0; // Convert ms to s
    lastControlTime = now;

    

    // Encoder Reading
    uint16_t angleRaw = encoder_RF.rawAngle();
    float currentAngle = (angleRaw * 360.0 / 4096.0);
    if (currentAngle < 0) currentAngle += 360.0;
    if (currentAngle >= 360.0) currentAngle -= 360.0;

    static bool firstRun = true;
    if (firstRun) {
      previousAngle = currentAngle;
      firstRun = false;
      return;
    }

    // Unwrap and compute angular velocity
    delta = currentAngle - previousAngle;
    if (delta > 180.0) delta -= 360.0;
    if (delta < -180.0) delta += 360.0;
    previousAngle = currentAngle;

    measured = -delta / dt;  // deg/s
    error = setpoint - measured;
    integral += error * dt;
    derivative = (error - last_Error) / dt;
    last_Error = error;

    if (isnan(delta) || isnan(currentAngle)) return;

    integral = constrain(integral, -1000.0, 1000.0); // ANTI-WINDUP-->I am struggling with determining this boundry

    float output = kp * error + ki * integral + kd * derivative;  // Control Signal u(t)

    // Convert control signal to PWM (0-255)
    pwm = constrain(abs(output), 0, 255);
    if (pwm < 20 && setpoint != 0) pwm = 20;
    if (setpoint == 0) pwm = 0;

    dir = (output >= 0) ? 'F' : 'R';

    setMotor(dir, pwm);  // Apply control signal to motor
  }

  // Print to the Serial Monitor
  if (now - lastPrintTime >= printInterval) {
    lastPrintTime = now;
    Serial.print("Setpoint: ");
    Serial.print(setpoint);
    Serial.print(" deg/s | Measured: ");
    Serial.print(measured);
    Serial.print(" deg/s | Error: ");
    Serial.print(error);
    Serial.print(" | PWM: ");
    Serial.print(pwm);
    Serial.print(" | Direction: ");
    Serial.print(dir);
    Serial.print(" | Delta: ");
    Serial.println(delta);
  }
}

// Motor control function
void setMotor(char dir, int pwmVal) {
  analogWrite(PWM1, pwmVal);    // Set the desired speed
  if (dir == 'F' || dir == 'f') {
    // Forward Rotation
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (dir == 'R' || dir == 'r') {
    // Reverse Rotation
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
}
