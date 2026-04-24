#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H


#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

// --- PID Gains (Initial Rad/s Estimates) ---
typedef struct{
    // --- Configuration ---
    float kp, ki, kd;             // Gains
    float kff;                    // feedforward gain (= 1/K = 1/0.2291 = 4.366)
    float output_min, output_max; // Output limits (e.g., -100 to 100)
    float dt;                     // Control interval in seconds 
    float deadband;               // % power offset

    // --- State Variables ---
    float setpoint;               // Target velocity
    float last_error;             // For Derivative term
    float integral_sum;           // For Integral term
}pid_ctrl_t;

// --- Function Prototypes ---
void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float kff,
              float deadband, float output_min, float output_max, float dt);
float pid_compute(pid_ctrl_t *pid, float measured_value);
void pid_reset(pid_ctrl_t *pid);

#ifdef __cplusplus
}
#endif

#endif