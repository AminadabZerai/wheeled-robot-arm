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