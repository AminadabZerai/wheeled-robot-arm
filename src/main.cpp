#include <Arduino.h>
#include <Wire.h>
#include "motor.h"
#include "as5600.h"
#include "imu.h"
#include "config.h"
#include "pid_controller.h"
#include "post.h"

unsigned long last_control_time = 0;
String serial_buffer = "";
bool setpoint_received = false;

motor_t motor_LF;
lsm6ds3_t imu;
as5600_t enc_LF;
sensor_data_t gyro_data;
sensor_data_t accel_data;
pid_ctrl_t pid_LF;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.setTimeout(50);
    while (!Serial);

    delay(5000);
    post_init();

    bool health = check_i2c();
    post_system_report(health);

    motor_init(&motor_LF, PIN_LF_PWM, PIN_LF_IN1, PIN_LF_IN2, PIN_STANDBY);
    lsm6ds3_init(&imu, LSM6DS3_ADDR);
    as5600_init(&enc_LF, MUX_CH_LF);
    pid_init(&pid_LF, PID_KP, PID_KI, PID_KD, PID_KFF,
             PID_DEADBAND, PID_OUT_MIN, PID_OUT_MAX, PID_DT);

    if (!lsm6ds3_check_id(&imu)) {
        Serial.println(F("IMU Authentication Failed!"));
        while(1) {
            digitalWrite(LED_BUILTIN, HIGH); delay(50);
            digitalWrite(LED_BUILTIN, LOW);  delay(50);
        }
    }
    Serial.println(F("IMU OK"));

    lsm6ds3_enable_accel(&imu);
    lsm6ds3_enable_gyro(&imu);

    Serial.println(F("Calibrating... Keep Robot Still."));
    uint32_t end_cal = millis() + 10000;
    while (millis() <= end_cal) {
        digitalWrite(LED_BUILTIN, HIGH);
        lsm6ds3_calibrate_accel(&imu);
        lsm6ds3_calibrate_gyro(&imu);
    }
    Serial.println(F("READY"));
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    // 1. NON-BLOCKING SERIAL COMMAND
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            if (serial_buffer.length() > 1) {
                char  cmd = serial_buffer.charAt(0);
                float val = serial_buffer.substring(2).toFloat();

                switch (cmd) {
                    case 'S':
                        pid_LF.setpoint = val;
                        setpoint_received = true;
                        if (fabsf(val) < 0.01f) {
                            pid_reset(&pid_LF);
                            setpoint_received = false;
                        }
                        break;
                    case 'P':
                        pid_LF.kp = val;
                        pid_reset(&pid_LF);
                        break;
                    case 'I':
                        pid_LF.ki = val;
                        pid_reset(&pid_LF);
                        break;
                    case 'D':
                        pid_LF.kd = val;
                        pid_reset(&pid_LF);
                        break;
                    case 'F':
                        pid_LF.kff = val;
                        pid_reset(&pid_LF);
                        break;
                    case 'R':
                        pid_reset(&pid_LF);
                        setpoint_received = false;
                        motor_set_speed(&motor_LF, 0.0f);
                        break;
                }
            }
            serial_buffer = "";
        } else {
            serial_buffer += c;
        }
    }

    // 2. TIMED CONTROL LOOP
    unsigned long current_time = millis();
    if (current_time - last_control_time >= CONTROL_INTERVAL_MS) {
        last_control_time = current_time;

        as5600_get_velocity(&enc_LF);
        float current_velocity = enc_LF.angular_velocity;

        if (setpoint_received) {
            float target_power = pid_compute(&pid_LF, current_velocity);
            motor_set_speed(&motor_LF, target_power);
        } else {
            motor_set_speed(&motor_LF, 0.0f);
        }

        // Telemetry: Time_ms, Setpoint, Velocity, Power, Kp, Ki, Kd
        Serial.print(millis());                  Serial.print(",");
        Serial.print(pid_LF.setpoint, 2);        Serial.print(",");
        Serial.print(current_velocity, 4);       Serial.print(",");
        Serial.print(motor_LF.current_power, 2); Serial.print(",");
        Serial.print(pid_LF.kp, 4);              Serial.print(",");
        Serial.print(pid_LF.ki, 4);              Serial.print(",");
        Serial.println(pid_LF.kd, 4);
    }
}