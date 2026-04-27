/* ============================================================================
 * motor.h - TB6612FNG Motor Driver Interface
 * ============================================================================
 * Exposes the public API for the brushed DC motor driver. Defines the
 * motor_t state struct and function prototypes used by main.cpp and the
 * PID control loop. Power is expressed as a percentage (-100 to +100)
 * rather than raw PWM to decouple control logic from hardware specifics.
 *
 * Target:   Arduino Nano 33 IoT
 * Driver:   TB6612FNG dual H-bridge
 * Protocol: Digital GPIO (IN1, IN2, STBY) + PWM (analogWrite)
 * Voltage:  7.4V motor supply, 3.3V logic
 *
 * Pin Roles:
 *   motor_pwm_pin    — PWM output controlling motor speed (0–255)
 *   motor_pin1       — H-bridge IN1 (direction bit A)
 *   motor_pin2       — H-bridge IN2 (direction bit B)
 *   motor_enable_pin — STBY pin (HIGH = enabled, LOW = standby)
 *
 * Truth Table:
 *   IN1   IN2   PWM    Mode
 *   HIGH  LOW   > 0    Forward
 *   LOW   HIGH  > 0    Reverse
 *   LOW   LOW   any    Coast (free spin)
 *   HIGH  HIGH  255    Brake (short-circuit, max torque)
 *
 * Deadband:
 *   motor_set_speed() adds MOTOR_DEADBAND (30%) to any non-zero command
 *   before sending to hardware. The PID controller outputs values in the
 *   range [-70, +70] and the driver maps these to [±30, ±100] at the motor.
 *   Do NOT add deadband in pid_compute() — double compensation causes
 *   severe overshoot.
 *
 * Usage:
 *   motor_t motor_LF;
 *   motor_init(&motor_LF, PIN_LF_PWM, PIN_LF_IN1, PIN_LF_IN2, PIN_STANDBY);
 *   motor_set_speed(&motor_LF, 45.0f);   // 45% forward
 *   motor_set_speed(&motor_LF, -30.0f);  // 30% reverse
 *   motor_brake(&motor_LF);              // active brake
 *
 * ============================================================================ */
 
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Structs Related to Motor
typedef struct {
    uint8_t motor_pwm_pin;
    uint8_t motor_pin1;
    uint8_t motor_pin2;
    uint8_t motor_enable_pin;
    float current_power;
    bool is_inverted;    // Flip direction in software if wired backwards
} motor_t;


// Functions Related to Motor

// Initialize Motor
void motor_init(motor_t *motor, uint8_t pwm, uint8_t in1, uint8_t in2, uint8_t stby);

// Stop Motor
void motor_brake(motor_t *motor);

// Control Motor Speed
// Set motor speed from -100.0 (Full Reverse) to 100.0 (Full Forward)
void motor_set_speed(motor_t *motor, float power);

// Standby Motor
void motor_standby(motor_t *motor, bool stby);


#ifdef __cplusplus
}
#endif

#endif