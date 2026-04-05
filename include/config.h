/* ============================================================================
   ROBOT CONFIGURATION - config.h (Pure PID / Radians / 7.4V)
   ============================================================================ */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Constants & Conversions ---
const float RAW_TO_RAD = (2.0f * PI) / 4096.0f; 
// const float RAD_TO_DEG = 57.29577951f;  // Provided by Arduino

// --- PID Gains (Initial Rad/s Estimates) ---
struct ControlGains {
    float kp = 36.5f;   
    float ki = 17.5f;   
    float kd = 0.45f;   
    float alpha = 0.25f; // LPF for velocity measurement
};

// --- Physical Limits ---
const int   PWM_MAX = 255;
const int   PWM_MIN = 30;     
const float VEL_MAX_RAD = 7.0f; // ~400 deg/s

// --- Device Addresses ---
const int AS5600_ADDR = 0x36;
const int MUX_ADDR = 0x70;

// -- Shared Registers ---
#define AS5600_STATUS_REG 0x0B
#define AS5600_GAIN_CONTROL_REG 0x1A


// --- System Timing ---
const float DT = 0.020f; // Seconds
#define I2C_CLOCK     400000 // 400kHz Fast Mode
#define I2C_CLOCK_STD 100000 // 100kHz Standard Mode
#define CONTROL_INTERVAL_MS 20 // 50Hz

// --- Hardware Pin Mapping ---

// Commmon Standby Pins for motors
#define PIN_STANDBY 21
// Left Front (LF)
#define PIN_LF_PWM 2
#define PIN_LF_IN1 4
#define PIN_LF_IN2 5
#define MUX_CH_LF  0

// Left Back (LB)
#define PIN_LB_PWM 3
#define PIN_LB_IN1 6
#define PIN_LB_IN2 7
#define MUX_CH_LB  1

// Right Front (RF)
#define PIN_RF_PWM 9
#define PIN_RF_IN1 8
#define PIN_RF_IN2 10
#define MUX_CH_RF  2

// Right Back (RB)
#define PIN_RB_PWM 11
#define PIN_RB_IN1 12
#define PIN_RB_IN2 20
#define MUX_CH_RB  3


#endif