/* ============================================================================
 * pid_controller.c - Reentrant PID + Feedforward Velocity Controller
 * ============================================================================
 * Description:
 * Implementation of a discrete-time PI controller with feedforward
 * compensation for closed-loop wheel velocity control on a mecanum
 * wheeled robot. Designed to work in conjunction with the AS5600 encoder
 * driver (velocity measurement) and the motor driver (power output).
 *
 * Hardware Target:
 * - Arduino Nano 33 IoT
 * - Control plant: brushed DC motor + gearbox + AS5600 encoder
 * - Control interval: 20ms (50Hz)
 *
 * Design Pattern:
 * - Reentrant procedural interface using (pid_ctrl_t *pid) pointers.
 *   One struct instance per wheel — supports 4 independent controllers
 *   without duplicating any logic.
 * - pid_init() calls pid_reset() to zero all state fields — prevents
 *   uninitialised integral or derivative state on first compute cycle.
 * - pid_reset() called on zero setpoint command and on every live gain
 *   change to prevent integral carryover between operating conditions.
 *
 * Control Law:
 *   ff_term  = sign(setpoint) * Kff * |setpoint|
 *   p_term   = Kp * error
 *   i_term   = Ki * integral_sum
 *   d_term   = Kd * (error - last_error) / dt
 *   output   = ff_term + p_term + i_term + d_term
 *
 * Feedforward Design:
 * - Kff = 1/K where K is the plant gain (rad/s per % power) from open-loop
 *   identification. Provides the baseline power needed to reach the setpoint
 *   without relying on the integrator, reducing settling time by ~14x.
 * - Deadband is NOT included in ff_term — motor_set_speed() applies the
 *   hardware deadband offset internally. Including it here causes double
 *   compensation and 22–44% overshoot.
 * - Sign-preserving formula handles reverse setpoints correctly.
 *
 * Anti-Windup:
 * - Clamping method: integral only accumulates when output is within
 *   [output_min, output_max]. Prevents windup during stall or saturation.
 * - Critical for Ki=10 with a 20ms interval — without clamping, one second
 *   of stall accumulates Ki * stall_error * 50 = potentially large values.
 *
 * Integral Formulation:
 * - integral_sum += error * dt (Riemann sum approximation)
 * - dt multiplication ensures accumulation rate is control-interval-
 *   independent and physically meaningful (units: rad).
 *
 * Tuning History:
 * - Model gains (pidtune): Kp=6.25, Ki=113 — failed on hardware due to
 *   deadband nonlinearity and missing dt in original implementation.
 * - Final empirical gains: Kp=10, Ki=10, Kd=0, Kff=4.366
 *   Settling: 0.04s | Overshoot: 8.2% | SS Error: <2% across 7-15 rad/s
 *
 * Dependencies:
 * - pid_controller.h
 * - config.h (PID_KP, PID_KI, PID_KD, PID_KFF, PID_DEADBAND, PID_DT)
 *
 * Reference:
 * - Brian Douglas, Control System Lectures (YouTube):
 *   https://youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y
 *
 * Author: Aminadab Z. Ghebrehiwet
 * Date:   2026-04-13 (initial) | 2026-04-24 (feedforward added)
 * ============================================================================ */
 
#include "pid_controller.h"


// Initialize PID controller
void pid_init(pid_ctrl_t *pid, float kp, float ki, float kd, float kff,
              float deadband, float output_min, float output_max, float dt) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kff = kff;
    pid->deadband = deadband;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->dt = dt;
    pid_reset(pid);
}

// Reset PID controller
void pid_reset(pid_ctrl_t *pid) {
    pid->setpoint = 0.0;
    pid->last_error = 0.0;
    pid->integral_sum = 0.0;
}


// Compute PID output
float pid_compute(pid_ctrl_t *pid, float measured_value) {
    float error = pid->setpoint - measured_value;

    // Feedforward — baseline power from setpoint directly
    float ff_term = 0.0f;
    if (fabsf(pid->setpoint) > 0.01f) {
        float sign = (pid->setpoint > 0.0f) ? 1.0f : -1.0f;
        ff_term = sign * pid->kff * fabsf(pid->setpoint);
    }

    // Proportional term
    float p_term = pid->kp * error;

    // Integral Term
    float i_term = pid->ki * pid->integral_sum;

    // Derivative Term
    float d_term = pid->kd * (error - pid->last_error) / pid->dt;
    
    
    float output = ff_term + p_term + i_term + d_term;

    // PID Anti-Windup (With Clamping Method)
    if (output > pid->output_max) {
        output = pid->output_max;
    }
    else if (output < pid->output_min) {
        output = pid->output_min;
    }
    else {
        // Only update integral accumulator if output is within limits
        pid->integral_sum += error * pid->dt;
    }

    // Save state for next iteration
    pid->last_error = error;
    return output;
}

