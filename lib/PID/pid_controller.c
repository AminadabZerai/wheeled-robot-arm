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

