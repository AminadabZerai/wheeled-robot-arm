/* ============================================================================
 * pid_controller.h - Reentrant PID + Feedforward Controller Interface
 * ============================================================================
 * Exposes the public API for the discrete-time PI+feedforward velocity
 * controller. Defines the pid_ctrl_t state struct and function prototypes
 * used by main.cpp to drive each wheel independently.
 *
 * Target:   Arduino Nano 33 IoT
 * Plant:    Brushed DC motor + gearbox measured by AS5600 encoder
 * Interval: 20ms (50Hz) — set via CONTROL_INTERVAL_MS in config.h
 * Voltage:  7.4V supply
 *
 * Control Law:
 *   ff    = sign(sp) * Kff * |sp|       — feedforward baseline
 *   p     = Kp * error                  — proportional correction
 *   i     = Ki * integral_sum           — integral correction
 *   d     = Kd * (error - last) / dt    — derivative correction (Kd=0)
 *   out   = ff + p + i + d              — total output (% power)
 *
 * Feedforward Note:
 *   Kff = 1/K where K is the open-loop plant gain (rad/s per % power).
 *   For this system: K = 0.2291 → Kff = 4.366.
 *   Deadband is NOT included here — motor_set_speed() handles it.
 *   Including deadband in ff_term causes double compensation (+44%
 *   overshoot observed in testing).
 *
 * Anti-Windup:
 *   Integral accumulates only when output is within [output_min, output_max].
 *   Prevents runaway accumulation during motor stall or output saturation.
 *
 * Live Gain Update:
 *   Gains (kp, ki, kd, kff) can be written directly to the struct at runtime.
 *   Always call pid_reset() after changing gains to prevent integral carryover.
 *
 * Final Tuned Values (single wheel, 7.4V, 7–15 rad/s operating range):
 *   Kp  = 10.0    Ki = 10.0    Kd = 0.0    Kff = 4.366
 *   Settling: 0.04s | Overshoot: 8.2% | SS Error: < 2%
 *
 * Usage:
 *   pid_ctrl_t pid_LF;
 *   pid_init(&pid_LF, PID_KP, PID_KI, PID_KD, PID_KFF,
 *            PID_DEADBAND, PID_OUT_MIN, PID_OUT_MAX, PID_DT);
 *   pid_LF.setpoint = 10.0f;
 *   float power = pid_compute(&pid_LF, enc_LF.angular_velocity);
 *   motor_set_speed(&motor_LF, power);
 *
 * ============================================================================ */
 
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