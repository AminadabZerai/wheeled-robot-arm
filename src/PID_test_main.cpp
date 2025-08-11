/* TB6612FNG DUAL MOTOR DRIVER AND AS5600 MAGNETIC ENCODER PID CONTROL TEST

 * TEST SETUP - Arduino Nano 33 IoT 3.3V Logic, Motor Driven at 7.4V
 * Direction and desired postition will be given to the motor throuhg serial input and
 * a PID Controller with tunable paramters Kp,Ki,Kd will be implemented to see how well the motor
 * responds to a given set-value.
 * Aminadab Z. Gherbehiwet - 5/8/2025
 */
#include <Arduino.h>

#include <Wire.h>
#include <math.h>
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


// Rotation tracking
float previousAngle = 0;


// PID Variables
float setpoint = 0;
float measured = 0;
float error = 0;
float integral = 0;
float derivative = 0;
float last_Error = 0;

// Control Variables
double dt;    // Time difference in seconds
char dir;     // Direction of rotation
float delta = 0.0;  // Change in angle for angular velocity calculation 


// PWM Variables
int pwm;      // PWM value for motor control
const int pwmMax = 255; // Maximum PWM value
const int pwmMin = 20;   // Minimum PWM value
const float tiny_setpoint = 3.0; // Threshold for small setpoint
const float tiny_error = 5.0; // Threshold for small error



// PID Tuning Parameters (Change these to tune the PID controller)
float kp = 0.6;
float ki = 0.3;
float kd = 0.003;

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

  Serial.println("PID Test Started");
  Serial.println("Kp: " + String(kp) + ", Ki: " + String(ki) + ", Kd: " + String(kd));
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
      setpoint = (direction == 'r' || direction == 'R') ? -fabs(speed) : fabs(speed);
      Serial.print("Received Setpoint: ");
      Serial.print(setpoint);
      Serial.println(" deg/s");
    } else {
      Serial.println("Invalid format. Use f,100 or r,90");
    }
  }

  // Control loop timing and control interval
  static unsigned long last_us = micros();
  unsigned long now_us = micros();
  unsigned long elapsed_us = now_us - last_us;

  // run control every 'controlInterval' ms
  if (elapsed_us < (unsigned long)controlInterval * 1000UL) {
    return;  // not time yet
  }

  dt = elapsed_us * 1e-6;     // seconds, ~0.020 s normally
  last_us = now_us;

    

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


    // PID Control Logic
    measured = -delta / dt;  // deg/s
    float alpha = 0.15;  // LPF smoothing factor
    static float filtered_measured = 0;

    filtered_measured = alpha * measured + (1 - alpha) * filtered_measured;
    measured = filtered_measured;
    
    error = setpoint - measured;

    static float last_measured = 0.0;
    derivative = -(measured - last_measured) / dt;  //Increase in measured speed over time should reduce the control signal (reacts to disturbances only and ensures smoother control)
    // derivative = (error - last_Error) / dt;  // Change in error over time  //--> reacts both to disturbances and changes in setpoint which ultimately leads to oscillations on setpoint changes
    //last_Error = error;
    last_measured = measured;

  

    if (isnan(delta) || isnan(currentAngle) || isnan(measured)) {
      setMotor('S', 0);  // S = stop both pins LOW
      return;
    }

    float P = kp * error;  // Proportional term
    float I = ki * integral;  // Integral term
    float D = kd * derivative;  // Derivative term   

    float output = P + I + D;  // PID Control Signal

    // Map control signal to PWM (0-255)
    int pwm_estimate = constrain((int)fabs(output), 0, 255); // Estimate PWM value before conditional integration check
    bool saturated = (pwm_estimate >= 255-1); // Check if PWM is saturated


   // Saturation Aware Integral for stronger anti-windup
    if (!saturated || (error * integral < 0)) {
      integral += error * dt;  // Update integral only if not saturated or if error and integral have opposite signs
      integral = constrain(integral, -300.0, 300.0); // Anti-windup limit
    } 
    // Recompute with updated I
    I = ki * integral;
    output = P + I + D;

    pwm = (int)fabs(output);  // Convert to PWM value
    pwm = constrain(pwm, 0, pwmMax);  // Ensure PWM is within valid range
    if (fabs(setpoint) < tiny_setpoint && fabs(error) < tiny_error) {
        pwm = 0;                 // let it settle, no twitch
      } else if (pwm > 0 && pwm < pwmMin) {
        pwm = pwmMin;           // overcome static friction only when needed
      }

    dir = (output >= 0) ? 'F' : 'R';
    
    if (pwm == 0) {
      dir = 'S';  // Stop if PWM is zero
    }
    setMotor(dir, pwm);  // Apply control signal to motor


  // Print to the Serial Monitor
  if (now - lastPrintTime >= printInterval) {
    lastPrintTime = now;
    Serial.print(millis()/1000.0); Serial.print('\t');
    Serial.print(setpoint);        Serial.print('\t');
    Serial.print(measured);        Serial.print('\t');
    Serial.print(error);           Serial.print('\t');
    Serial.print(pwm);             Serial.print('\t');
    Serial.println(delta);
    // Serial.print("Setpoint: ");
    // Serial.print(setpoint);
    // Serial.print(" deg/s | Measured: ");
    // Serial.print(measured);
    // Serial.print(" deg/s | Error: ");
    // Serial.print(error);
    // Serial.print(" | PWM: ");
    // Serial.print(pwm);
    // Serial.print(" | Direction: ");
    // Serial.print(dir);
    // Serial.print(" | Delta: ");
    // Serial.println(delta);
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
