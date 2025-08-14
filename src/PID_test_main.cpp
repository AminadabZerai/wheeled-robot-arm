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
/*
These values are the best Tuning values for the PID parameters for the motor and encoder setup and were obtained through trial and error
--> Kp = 0.64, Ki = 0.31, Kd = 0.0072
--> These values are tuned for a 7.4V motor and a 3.3V logic Arduino Nano 33 IoT
 If you change the motor voltage or the Arduino logic voltage, you will need to retune the PID parameters
 */
float kp = 0.64;
float ki = 0.31; // Changed from the best 0.34 as we are integrating Feedback Error (Fe) to the integral term
float kd = 0.0078;


// Feedforward Variables
//-------------------------
/*
*These values are obtained from the PID tests where a CSV data was generated with the motor running at a constant speed when setpoints were given at a certain interval.
*These values are used to estimate the PWM value needed to achieve a certain speed given by the setpoint.
*Kv --> compensates backEMF and friction, which is proportional to the speed of the motor. It is the slope of the linear regression line that was fitted to the data points of the speed vs PWM values.
*Ku -->  it's the offset/intercept of the linear regression line that was fitted to the data points of the speed vs PWM values.
*/

/* -------------------------------------------
NB - These are tunable values and should be changed based on the motor and encoder setup you are using.

float Kv_pos = 0.1625f, Ku_pos = 9.7f;
float Kv_neg = 0.1608f, Ku_neg = 10.0f;
Values obtained from the PID tests with the motor running at a constant speed when setpoints were given at a certain interval.
Kp = 0.64, Ki = 0.34, Kd = 0.072
----------------------------------------------*/

float Kv_pos = 0.1588; // Feedforward gain for postive direction speed
float Kv_neg = 0.1591; // Feedforward gain for negative direction speed
float Ku_pos = 9.24; // Feedforward gain for control signal in positive direction 
float Ku_neg = 8.88; // Feedforward gain for negative direction control signal


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


    // PID Control Logic and LPF
    measured = -delta / dt;  // deg/s
    float alpha = 0.25;  // LPF smoothing factor
    static float filtered_measured = 0;

    filtered_measured = alpha * measured + (1 - alpha) * filtered_measured;
    measured = filtered_measured;
    
    error = setpoint - measured;

    static float last_measured = 0.0;
    derivative = -(measured - last_measured) / dt;  //Increase in measured speed over time should reduce the control signal (reacts to disturbances only and ensures smoother control)
    // derivative = (error - last_Error) / dt;  // Change in error over time  //--> reacts both to disturbances and changes in setpoint which ultimately leads to oscillations on setpoint changes
    //last_Error = error;
    last_measured = measured;

    // Reverse Bleed Management
    // If the setpoint is in the opposite direction of the measured speed, reduce the integral term to prevent overshoot
    // This is a simple anti-windup strategy to prevent integral windup when reversing direction  
    bool reversing = (setpoint * measured < 0.0f);
    bool near_zero = (fabs(measured) < 30.0f);
    if (reversing && near_zero) integral *= 0.5;   // brief I bleed

    

    if (isnan(delta) || isnan(currentAngle) || isnan(measured)) {
      setMotor('S', 0);  // S = stop both pins LOW
      return;
    }

    float P = kp * error;  // Proportional term
    float I = ki * integral;  // Integral term
    float D = kd * derivative;  // Derivative term
    
    // Feedforward Control
    float u_ff; // Feedforward control signal
    if (setpoint > 0.0){
      u_ff = Kv_pos * fabs(setpoint) + Ku_pos; // Feedforward for positive direction
    }
    else{
      u_ff = -(Kv_neg * fabs(setpoint) + Ku_neg); // Feedforward for negative direction}
    }
    if (fabs(setpoint) < tiny_setpoint) {
      u_ff = 0.0; // No feedforward for small setpoints
    } 

    float output = P + I + D + u_ff;  // PID Control Signal along with Feedforward

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
    output = P + I + D + u_ff;

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
