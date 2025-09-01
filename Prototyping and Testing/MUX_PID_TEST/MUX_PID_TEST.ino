/* ============================================================================
   PID + FEED-FORWARD VELOCITY CONTROL(AS5600 + TB6612FNG)
   Target   : Arduino Nano 33 IoT (3.3 V logic)
   Motor Pwr: 7.4 V (Li-ion/LiPo)
   Purpose  : Closed-loop velocity control with PID (D-on-measurement) and
              direction-specific feed-forward. Designed for step/ramp tests and
              easy extension to multi-motor systems.
   ============================================================================

   Hardware
   --------
   • Driver : TB6612FNG  (PWM=5, AIN1=3, AIN2=4; STBY tied HIGH)
   • Encoder: AMS AS5600 magnetic encoder on I²C (0x36), 12-bit (4096 counts/rev)
   • Optional future: I²C mux (e.g., TCA9548A) per motor/encoder channel

   Control Summary
   ---------------
   • Loop period        : 20 ms (≈50 Hz)
   • Velocity estimate  : unwrap angle; measured[deg/s] = −Δθ/Δt  (sign matches FWD)
   • Measurement filter : EMA low-pass, α = 0.25  (typical 0.15–0.35)
   • PID                : P on error, D on measurement (disturbance-oriented)
   • Feed-forward (FF)  : u_ff = sgn(setpoint) * (Kv*|setpoint| + Ku)
                           — separate Kv/Ku for + and − directions
   • Anti-windup        : conditional I update using *total* command saturation
   • Reversal handling  : halve integral near zero-crossing (|measured|<~30 deg/s);
                          optional short I “freeze” window (≈120 ms)
   • Actuator limits    : pwmMax=255; deadband lift pwmMin=20;
                          tiny_setpoint=3 deg/s, tiny_error=5 deg/s (zero PWM window)

   Default Tunings (proven baseline)
   ---------------------------------
   • Kp = 0.64, Ki = 0.31, Kd = 0.0078, α = 0.25
   • Kv_pos = 0.1588, Ku_pos = 9.06
     Kv_neg = 0.1612, Ku_neg = 8.43
   (If supply/load changes: refit Ku/Kv first, then fine-tune Ki/Kd.)

   Serial Interface
   ----------------
   • Command input : "f,150" (forward 150 deg/s), "r,90" (reverse 90 deg/s)
   • Log output    : every 500 ms → time_s, setpoint, measured, error, pwm, delta_deg

   Notes
   -----
   • Angle from AS5600 rawAngle(0..4095) → 0..360 deg; unwrap across 0/360.
   • D on measurement reduces setpoint-change kick; FF supplies steady torque.
   • For multi-motor: wrap this logic in a Motor class, stagger I²C reads, and
     store per-motor Kv/Ku in EEPROM (battery-voltage scaling recommended).

   Author   : Aminadab Z. Gherbehiwet
   Last Tuned: 2025-08-15
   ============================================================================ */

//#include <Arduino.h>

#include <Wire.h>
#include <math.h>
#include "AS5600.h"

// Pin definitions
// Front Left Motor
#define PWM2 9
#define BIN1 7
#define BIN2 8

// Front Right Motor
#define PWM1 5
#define AIN1 3
#define AIN2 4

AS5600 encoder_RF;    // Creating Encoder Object
AS5600 encoder_LF;

// Timing
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500;

unsigned long lastControlTime = 0;
const unsigned long controlInterval = 20;

// Rotation tracking
float previousAngle_LF = 0, previousAngle_RF = 0;

// PID Variables
float setpoint = 0;
float measured_LF = 0, measured_RF = 0;
float error_LF = 0, error_RF = 0;
float integral_LF = 0, integral_RF = 0;
float derivative_LF = 0, derivative_RF = 0;
float last_measured_LF = 0, last_measured_RF = 0;

// Control Variables
double dt;    // Time difference in seconds
char dir;     // Direction of rotation (kept for compatibility, not used for actuation)
float delta_LF = 0, delta_RF = 0;  // Change in angle for angular velocity calculation

// PWM Variables
int pwm;      // PWM value for motor control (kept for compatibility)
int pwm_LF = 0, pwm_RF = 0;  // Per-motor PWM values
const int pwmMax = 255; // Maximum PWM value
const int pwmMin = 20;   // Minimum PWM value
const float tiny_setpoint = 3.0; // Threshold for small setpoint
const float tiny_error = 5.0; // Threshold for small error

// PID Tuning Parameters (Change these to tune the PID controller)
/*
These values are the best Tuning values for the PID parameters for the motor and encoder setup and were obtained through trial and error
--> Kp = 0.64, Ki = 0.31, Kd = 0.0072 (Kd = 0.0078 for the best performance with Feedforward Control)
--> These values are tuned for a 7.4V motor and a 3.3V logic Arduino Nano 33 IoT
 If you change the motor voltage or the Arduino logic voltage, you will need to retune the PID parameters
 */
float kp = 0.64;
float ki = 0.31; // Changed from the best 0.34 as we are integrating Feedback Error (Fe) to the integral term
float kd = 0.0078;

// Measurement filter
const float alpha = 0.25;  // LPF smoothing factor
static float filtered_measured_LF = 0.0f;
static float filtered_measured_RF = 0.0f;

// Feedforward Variables
/*--------------------------------------------------------------------------------------------------------/
// Model: u_ff = sgn(SP) * (Kv_dir*|SP| + Ku_dir).
// Kv: PWM/(deg/s) for back-EMF/viscous; Ku: PWM offset for static friction.
// Use direction-specific {Kv_pos,Ku_pos}/{Kv_neg,Ku_neg}; set u_ff=0 when |SP|<tiny_setpoint.
// Anti-windup/saturation must use TOTAL = PID + u_ff.
// Baseline 2025-08-15 → Kv_pos=0.161, Ku_pos=7.8; Kv_neg=0.159, Ku_neg=8.8 Kp=0.64, Ki=0.31, Kd=0.0078; α=0.25.
----------------------------------------------------------------------------------------------------------
   Feed-forward gains for positive and negative directions.
   These values are tuned for the specific motor and encoder setup.
   Adjust these values if you change the motor or the encoder.
   The feed-forward gains are used to provide a baseline control signal
   that compensates for the motor's back-EMF and static friction. 
----------------------------------------------*/

float Kv_pos = 0.161; // Feedforward gain for postive direction speed
float Kv_neg = 0.159; // Feedforward gain for negative direction speed
float Ku_pos = 7.8; // Feedforward gain for control signal in positive direction 
float Ku_neg = 8.8; // Feedforward gain for negative direction control signal

// Function Prototypes
void setMotorRF(char dir, int pwmVal);
void setMotorLF(char dir, int pwmVal);
void muxSelect(uint8_t ch);
void scanMuxChannels();
void probeAnglesOnce();
inline uint16_t rawAngleOn(uint8_t ch, AS5600& enc);

void setup() {

  // Initializations
  Serial.begin(115200);
  while (!Serial);

  Wire.begin();
  Wire.setClock(400000); // optional: faster I2C

  // Set multiplexer to channel 0 and initialize Left Front Wheel Encoder with I2C
  muxSelect(0);
  encoder_LF.begin();
  // Set multiplexer to channel 3 and initialize Right Front Wheel Encoder with I2C
  muxSelect(3);
  encoder_RF.begin();

  scanMuxChannels();
  for (int i=0;i<5;i++){ probeAnglesOnce(); delay(300); } // rotate wheels by hand

  pinMode(PWM1, OUTPUT); pinMode(PWM2, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);

  // Zero the encoder
  // Prime previous angles once
  muxSelect(0); previousAngle_LF = (encoder_LF.rawAngle() * 360.0f / 4096.0f);
  muxSelect(3); previousAngle_RF = (encoder_RF.rawAngle() * 360.0f / 4096.0f);

  Serial.println("2-motor PID+FF ready. Use 'f,150' or 'r,90' (shared setpoint).");
  Serial.print("Kp="); Serial.print(kp);
  Serial.print(" Ki="); Serial.print(ki);
  Serial.print(" Kd="); Serial.print(kd);
  Serial.print("  Kv+/Ku+="); Serial.print(Kv_pos); Serial.print("/"); Serial.print(Ku_pos);
  Serial.print("  Kv-/Ku-="); Serial.print(Kv_neg); Serial.print("/"); Serial.println(Ku_neg);

  }

void loop() {
  unsigned long now = millis();

  // Read and parse user command
  if (Serial.available() > 0) {
    String user_cmd = Serial.readStringUntil('\n'); // read full line
    user_cmd.trim();                                // remove whitespaces and newline


    int comma_idx = user_cmd.indexOf(',');          // Locate comma
    if (comma_idx > 0 && user_cmd.length() > (size_t)(comma_idx + 1)) {
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
  // muxSelect(0); uint16_t angleRaw_LF = encoder_LF.rawAngle();
  // muxSelect(3); uint16_t angleRaw_RF = encoder_RF.rawAngle();

  uint16_t angleRaw_LF = rawAngleOn(0, encoder_LF);
  uint16_t angleRaw_RF = rawAngleOn(3, encoder_RF);

  float currentAngle_RF = (angleRaw_RF * 360.0f / 4096.0f);
  float currentAngle_LF = (angleRaw_LF * 360.0f / 4096.0f);
  if (currentAngle_RF < 0) currentAngle_RF += 360.0f;
  if (currentAngle_LF < 0) currentAngle_LF += 360.0f;
  if (currentAngle_RF >= 360.0f) currentAngle_RF -= 360.0f;
  if (currentAngle_LF >= 360.0f) currentAngle_LF -= 360.0f;

  static bool firstRun = true;
  if (firstRun) {
    previousAngle_LF = currentAngle_LF;
    previousAngle_RF = currentAngle_RF;
    firstRun = false;
    return;
  }

  // Unwrap and compute angular velocity
  delta_LF = currentAngle_LF - previousAngle_LF;
  if (delta_LF > 180.0f) delta_LF -= 360.0f;
  if (delta_LF < -180.0f) delta_LF += 360.0f;
  previousAngle_LF = currentAngle_LF;

  delta_RF = currentAngle_RF - previousAngle_RF;
  if (delta_RF > 180.0f) delta_RF -= 360.0f;
  if (delta_RF < -180.0f) delta_RF += 360.0f;
  previousAngle_RF = currentAngle_RF;

  // Reject impossible single-step jumps (protect D and I)
  const float maxStepDeg = 60.0f;  // ~3000 deg/s @ 20 ms; adjust if your dt changes
  if (fabs(delta_LF) > maxStepDeg) delta_LF = (delta_LF > 0 ? maxStepDeg : -maxStepDeg);
  if (fabs(delta_RF) > maxStepDeg) delta_RF = (delta_RF > 0 ? maxStepDeg : -maxStepDeg);

  // --- Median-of-3 on delta to kill single-sample outliers ---
  static float d1_LF = 0.0f, d2_LF = 0.0f, d1_RF = 0.0f, d2_RF = 0.0f;  // small history

  // shift histories
  d2_LF = d1_LF; d1_LF = delta_LF;
  d2_RF = d1_RF; d1_RF = delta_RF;

  // median helper
  auto med3 = [](float a, float b, float c){
    if (a > b) { float t=a; a=b; b=t; }
    if (b > c) { float t=b; b=c; c=t; }
    if (a > b) { float t=a; a=b; b=t; }
    return b; // median
  };

  float delta_LF_med = med3(delta_LF, d1_LF, d2_LF);
  float delta_RF_med = med3(delta_RF, d1_RF, d2_RF);

  // PID Control Logic and LPF
  measured_LF = -delta_LF_med / dt;  // deg/s
  measured_RF = delta_RF_med / dt;  // deg/s

  // After measured_* computation
  const float vmax = 400.0f;  // deg/s; choose safely above your real targets
  if (measured_LF >  vmax) measured_LF =  vmax;
  if (measured_LF < -vmax) measured_LF = -vmax;
  if (measured_RF >  vmax) measured_RF =  vmax;
  if (measured_RF < -vmax) measured_RF = -vmax;

  filtered_measured_LF = alpha * measured_LF + (1.0f - alpha) * filtered_measured_LF;
  filtered_measured_RF = alpha * measured_RF + (1.0f - alpha) * filtered_measured_RF;
  measured_RF = filtered_measured_RF;
  measured_LF = filtered_measured_LF;

  //============================= LEFT FRONT CONTROL ==================================

  error_LF = setpoint - measured_LF;

  //Increase in measured speed over time should reduce the control signal (reacts to disturbances only and ensures smoother control)
  derivative_LF = -(measured_LF - last_measured_LF) / dt;
  last_measured_LF = measured_LF;

  // Reverse Bleed Management
  // If the setpoint is in the opposite direction of the measured speed, reduce the integral term to prevent overshoot
  // This is a simple anti-windup strategy to prevent integral windup when reversing direction
  bool reversing_LF = (setpoint * measured_LF < 0.0f);
  bool near_zero_LF = (fabs(measured_LF) < 30.0f);
  if (reversing_LF && near_zero_LF) integral_LF *= 0.5f;   // brief I bleed

  if (isnan(delta_LF) || isnan(currentAngle_LF) || isnan(measured_LF)) {
    setMotorLF('S', 0);  // S = stop both pins LOW
    return;
  }

  float P_LF = kp * error_LF;      // Proportional term
  float I_LF = ki * integral_LF;   // Integral term
  float D_LF = kd * derivative_LF; // Derivative term

  // Feedforward Control
  float u_ff_LF; // Feedforward control signal
  if (setpoint > 0.0f){
    u_ff_LF = Kv_pos * fabs(setpoint) + Ku_pos; // Feedforward for positive direction
  } else {
    u_ff_LF = -(Kv_neg * fabs(setpoint) + Ku_neg); // Feedforward for negative direction
  }
  if (fabs(setpoint) < tiny_setpoint) {
    u_ff_LF = 0.0f; // No feedforward for small setpoints
  }

  float output_LF = P_LF + I_LF + D_LF + u_ff_LF;  // PID Control Signal along with Feedforward

  // Map control signal to PWM (0-255)
  int pwm_estimate_LF = constrain((int)fabs(output_LF), 0, 255); // Estimate PWM value before conditional integration check
  bool saturated_LF = (pwm_estimate_LF >= 255-1); // Check if PWM is saturated

  // Saturation Aware Integral for stronger anti-windup
  if (!saturated_LF || (error_LF * integral_LF < 0)) {
    integral_LF += error_LF * dt;  // Update integral only if not saturated or if error and integral have opposite signs
    integral_LF = constrain(integral_LF, -300.0f, 300.0f); // Anti-windup limit
  }
  // Recompute with updated I
  I_LF = ki * integral_LF;
  output_LF = P_LF + I_LF + D_LF + u_ff_LF;

  pwm_LF = (int)fabs(output_LF);  // Convert to PWM value
  pwm_LF = constrain(pwm_LF, 0, pwmMax);  // Ensure PWM is within valid range
  if (fabs(setpoint) < tiny_setpoint && fabs(error_LF) < tiny_error) {
    pwm_LF = 0;                 // let it settle, no twitch
  } else if (pwm_LF > 0 && pwm_LF < pwmMin) {
    pwm_LF = pwmMin;           // overcome static friction only when needed
  }

  //============================= RIGHT FRONT CONTROL ==================================

  error_RF = setpoint - measured_RF;

  //Increase in measured speed over time should reduce the control signal (reacts to disturbances only and ensures smoother control)
  derivative_RF = -(measured_RF - last_measured_RF) / dt;
  last_measured_RF = measured_RF;

  // Reverse Bleed Management
  // If the setpoint is in the opposite direction of the measured speed, reduce the integral term to prevent overshoot
  // This is a simple anti-windup strategy to prevent integral windup when reversing direction
  bool reversing_RF = (setpoint * measured_RF < 0.0f);
  bool near_zero_RF = (fabs(measured_RF) < 30.0f);
  if (reversing_RF && near_zero_RF) integral_RF *= 0.5f;   // brief I bleed

  if (isnan(delta_RF) || isnan(currentAngle_RF) || isnan(measured_RF)) {
    setMotorRF('S', 0);  // S = stop both pins LOW
    return;
  }

  float P_RF = kp * error_RF;      // Proportional term
  float I_RF = ki * integral_RF;   // Integral term
  float D_RF = kd * derivative_RF; // Derivative term

  // Feedforward Control
  float u_ff_RF; // Feedforward control signal
  if (setpoint > 0.0f){
    u_ff_RF = Kv_pos * fabs(setpoint) + Ku_pos; // Feedforward for positive direction
  } else {
    u_ff_RF = -(Kv_neg * fabs(setpoint) + Ku_neg); // Feedforward for negative direction
  }
  if (fabs(setpoint) < tiny_setpoint) {
    u_ff_RF = 0.0f; // No feedforward for small setpoints
  }

  float output_RF = P_RF + I_RF + D_RF + u_ff_RF;  // PID Control Signal along with Feedforward

  // Map control signal to PWM (0-255)
  int pwm_estimate_RF = constrain((int)fabs(output_RF), 0, 255); // Estimate PWM value before conditional integration check
  bool saturated_RF = (pwm_estimate_RF >= 255-1); // Check if PWM is saturated

  // Saturation Aware Integral for stronger anti-windup
  if (!saturated_RF || (error_RF * integral_RF < 0)) {
    integral_RF += error_RF * dt;  // Update integral only if not saturated or if error and integral have opposite signs
    integral_RF = constrain(integral_RF, -300.0f, 300.0f); // Anti-windup limit
  }
  // Recompute with updated I
  I_RF = ki * integral_RF;
  output_RF = P_RF + I_RF + D_RF + u_ff_RF;

  pwm_RF = (int)fabs(output_RF);  // Convert to PWM value
  pwm_RF = constrain(pwm_RF, 0, pwmMax);  // Ensure PWM is within valid range
  if (fabs(setpoint) < tiny_setpoint && fabs(error_RF) < tiny_error) {
    pwm_RF = 0;                 // let it settle, no twitch
  } else if (pwm_RF > 0 && pwm_RF < pwmMin) {
    pwm_RF = pwmMin;           // overcome static friction only when needed
  }

  // Decide directions from total outputs (before slew)
  char dir_LF = (output_LF >= 0.0f) ? 'F' : 'R';
  if (pwm_LF == 0) { dir_LF = 'S'; }
  char dir_RF = (output_RF >= 0.0f) ? 'F' : 'R';
  if (pwm_RF == 0) { dir_RF = 'S'; }

  // --- PWM slew limiting for both motors ---
  static int last_pwm_LF = 0, last_pwm_RF = 0;
  const int dPWMmax = 40;  // max change per 20 ms tick (tune 30–60)

  int target_LF = pwm_LF;
  if (target_LF > last_pwm_LF + dPWMmax) target_LF = last_pwm_LF + dPWMmax;
  if (target_LF < last_pwm_LF - dPWMmax) target_LF = last_pwm_LF - dPWMmax;
  pwm_LF = target_LF;
  last_pwm_LF = pwm_LF;

  int target_RF = pwm_RF;
  if (target_RF > last_pwm_RF + dPWMmax) target_RF = last_pwm_RF + dPWMmax;
  if (target_RF < last_pwm_RF - dPWMmax) target_RF = last_pwm_RF - dPWMmax;
  pwm_RF = target_RF;
  last_pwm_RF = pwm_RF;


  // Command BOTH motors with slewed PWMs
  setMotorLF(dir_LF, pwm_LF);
  setMotorRF(dir_RF, pwm_RF);

  // Print to the Serial Monitor
//  if (now - lastPrintTime >= printInterval) {
//    lastPrintTime = now;
//    
//    // Create a single, tab-separated string for easy parsing in Python
//    String logData = "";
//    logData += String(millis() / 1000.0, 2); // 1: Time (s)
//    logData += "\t";
//    logData += String(setpoint, 2);           // 2: Setpoint (deg/s)
//    logData += "\t";
//    logData += String(measured_LF, 2);       // 3: Measured LF (deg/s)
//    logData += "\t";
//    logData += String(pwm_LF);                 // 4: PWM LF
//    logData += "\t";
//    logData += String(delta_LF, 2);          // 5: Delta LF (deg)
//    logData += "\t";
//    logData += String(measured_RF, 2);       // 6: Measured RF (deg/s)
//    logData += "\t";
//    logData += String(pwm_RF);                 // 7: PWM RF
//    logData += "\t";
//    logData += String(delta_RF, 2);          // 8: Delta RF (deg)
//    logData += "\t";
//    logData += String(error_LF, 2);           // 9: Error LF (can use for both if setpoint is shared)
//
//    Serial.println(logData);
//  }
//}

  // Print to the Serial Monitor
   if (now - lastPrintTime >= printInterval) {
     lastPrintTime = now;
     Serial.print(millis()/1000.0); Serial.print('\t');
     Serial.print(" SETPOINT"); Serial.print(setpoint);
     Serial.print(" MEAS_LF="); Serial.print(measured_LF);
     Serial.print(" PWM_LF="); Serial.print(pwm_LF);
     Serial.print(" DELTA_LF="); Serial.print(delta_LF);
     Serial.print('\t');
     Serial.print(" MEAS_RF="); Serial.print(measured_RF);
     Serial.print(" PWM_RF="); Serial.print(pwm_RF);
     Serial.print(" DELTA_RF="); Serial.println(delta_RF);

  

    // Serial.print(millis()/1000.0); Serial.print('\t');
    // Serial.print(setpoint);        Serial.print('\t');
    // Serial.print(measured);        Serial.print('\t');
    // Serial.print(error);           Serial.print('\t');
    // Serial.print(pwm);             Serial.print('\t');
    // Serial.println(delta);

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
void setMotorRF(char dir, int pwmVal) {            // Right/Front (A-channel)
  analogWrite(PWM1, pwmVal);
  if (pwmVal == 0) { digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW); return; }
  if (dir=='F'||dir=='f'){ digitalWrite(AIN1,HIGH); digitalWrite(AIN2,LOW); }
  else                   { digitalWrite(AIN1,LOW);  digitalWrite(AIN2,HIGH); }
}
void setMotorLF(char dir, int pwmVal) {            // Left/Front (B-channel)
  analogWrite(PWM2, pwmVal);
  if (pwmVal == 0) { digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW); return; }
  if (dir=='F'||dir=='f'){ digitalWrite(BIN1,HIGH); digitalWrite(BIN2,LOW); }
  else                   { digitalWrite(BIN1,LOW);  digitalWrite(BIN2,HIGH); }
}

inline void muxSelect(uint8_t ch) {
  Wire.beginTransmission(0x70);     // TCA9548A Address
  Wire.write(1u << (ch & 7));
  Wire.endTransmission();
}

void scanMuxChannels() {
  Serial.println("I2C scan per TCA9548A channel (expect AS5600 at 0x36):");
  for (uint8_t ch = 0; ch < 8; ++ch) {
    muxSelect(ch); delay(2);
    Wire.beginTransmission(0x36);
    uint8_t err = Wire.endTransmission();
    Serial.print("  CH"); Serial.print(ch); Serial.print(": ");
    Serial.println(err == 0 ? "FOUND 0x36" : "-");
  }
  muxSelect(0);
}
void probeAnglesOnce() {
  muxSelect(0); uint16_t raw0 = encoder_LF.rawAngle();
  muxSelect(3); uint16_t raw3 = encoder_RF.rawAngle();
  Serial.print("LF ch0 raw="); Serial.print(raw0);
  Serial.print("  RF ch3 raw="); Serial.println(raw3);
}

// Stable rawAngle read on a given mux channel
inline uint16_t rawAngleOn(uint8_t ch, AS5600& enc) {
  muxSelect(ch);
  delayMicroseconds(600);          // a bit more settle time than 300
  (void)enc.rawAngle();            // dummy 1
  (void)enc.rawAngle();            // dummy 2
  uint16_t a = enc.rawAngle();     // sample A
  uint16_t b = enc.rawAngle();     // sample B
  // accept if within 2 LSB (~0.18 deg); otherwise trust the second one
  if ((a > b ? a - b : b - a) <= 2) return a;
  return b;
}
