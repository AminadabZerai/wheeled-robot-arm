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