/* ============================================================================
 * motor.cpp - TB6612FNG Motor Driver Hardware Abstraction Layer
 * ============================================================================
 * Description:
 * Procedural driver for brushed DC motors controlled via the TB6612FNG
 * dual H-bridge motor driver. Handles direction control, PWM generation,
 * deadband compensation, output clamping, coast, and active braking.
 * Designed to be driven directly by the PID controller output.
 *
 * Hardware Target:
 * - Arduino Nano 33 IoT
 * - Driver IC: TB6612FNG (dual H-bridge)
 * - Control signals: IN1, IN2 (direction), PWM (speed), STBY (enable)
 * - Power supply: 7.4V 2S LiPo
 *
 * Design Pattern:
 * - Reentrant procedural interface using (motor_t *motor) pointers.
 * - Power represented as percentage (-100.0 to +100.0) — decouples
 *   control logic from raw PWM hardware, making PID output human-readable
 *   and portable across different MCUs or driver ICs.
 * - PWM conversion (0–255) isolated to the lowest level in motor_set_speed().
 * - Deadband compensation applied before clamping — ensures final_power
 *   never exceeds output limits after the offset is added.
 * - All direction checks and PWM calculation use final_power (post-deadband),
 *   not the original power argument — prevents erratic behaviour at the
 *   deadband boundary where direction and magnitude would otherwise mismatch.
 *
 * Operating Modes:
 * - Forward:  IN1=HIGH, IN2=LOW,  PWM > 0
 * - Reverse:  IN1=LOW,  IN2=HIGH, PWM > 0
 * - Coast:    IN1=LOW,  IN2=LOW,  any PWM  (motor free-spins)
 * - Brake:    IN1=HIGH, IN2=HIGH, PWM=255  (short-circuit braking torque)
 * - Standby:  STBY=LOW (disables entire driver IC, both channels)
 *
 * Key Design Decisions:
 * - is_inverted flag reserved for software direction correction when a
 *   motor is physically wired backwards — avoids rewiring on assembly.
 * - STBY pin driven per-command based on final_power magnitude rather
 *   than a separate enable call — ensures motor is always in the correct
 *   state regardless of call order.
 *
 * Dependencies:
 * - motor.h
 * - config.h (MOTOR_DEADBAND)
 *
 * Author: Aminadab Z. Ghebrehiwet
 * Date:   2026-04-04
 * ============================================================================ */
#include "motor.h"

// Initialize motor
void motor_init(motor_t *motor, uint8_t pwm, uint8_t in1, uint8_t in2, uint8_t stby) {
    motor->motor_pwm_pin = pwm;
    motor->motor_pin1 = in1;
    motor->motor_pin2 = in2;
    motor->motor_enable_pin = stby;
    motor->current_power = 0.0f;
    motor->is_inverted = false;

    // Set pin modes
    pinMode(motor->motor_pwm_pin, OUTPUT);
    pinMode(motor->motor_pin1, OUTPUT);
    pinMode(motor->motor_pin2, OUTPUT);
    pinMode(motor->motor_enable_pin, OUTPUT);

    // Ensure motor starts OFF
    motor_standby(motor, false); 
    motor_set_speed(motor, 0.0f);
}

// Set motor power
void motor_set_speed(motor_t *motor, float power) {

    // Deadband Compensation
    float final_power = power;

    if (power > 0.1f) {
        final_power = power + MOTOR_DEADBAND;
    } else if (power < -0.1f) {
        final_power = power - MOTOR_DEADBAND;
    }
    else {
        final_power = 0.0f;
    }
    // Clamp speed
    if (final_power > 100.0f) {
        final_power = 100.0f;
    } else if (final_power < -100.0f) {
        final_power = -100.0f;
    }
    motor->current_power = final_power;

    // Enable motor
    digitalWrite(motor->motor_enable_pin, 
        (fabs(final_power) > 0.1f) ? HIGH : LOW); 

    // Set Power based on direction
    if (final_power > 0.1){
        digitalWrite(motor->motor_pin1, HIGH);
        digitalWrite(motor->motor_pin2, LOW);   
    }
    else if (final_power < -0.1){
        digitalWrite(motor->motor_pin1, LOW);
        digitalWrite(motor->motor_pin2, HIGH);
    }
    else { // Let the motor coast
        digitalWrite(motor->motor_pin1, LOW);
        digitalWrite(motor->motor_pin2, LOW);
    }

    // Calculate PWM
    uint8_t pwm = (uint8_t)map(fabs(power), 0.0f, 100.0f, 0, 255);
    analogWrite(motor->motor_pwm_pin, pwm);

}

// Standby motor
void motor_standby(motor_t *motor, bool standby) {
    // STBY is Active HIGH (High = On, Low = Standby/Off)
    digitalWrite(motor->motor_enable_pin, standby ? LOW : HIGH);
}

// Stop motor
void motor_brake(motor_t *motor) {
    digitalWrite(motor->motor_pin1, HIGH);
    digitalWrite(motor->motor_pin2, HIGH);
    analogWrite(motor->motor_pwm_pin, 255); // Full braking torque
}