#include <Arduino.h>
#include <Wire.h>
#include "motor.h"
#include "as5600.h"
#include "imu.h"
#include "config.h"
#include "post.h"

unsigned long last_print_time = 0;
String serial_buffer = "";

motor_t motor_LF;
lsm6ds3_t imu;
as5600_t enc_LF;
sensor_data_t gyro_data;
sensor_data_t accel_data;

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
    // Non-blocking serial command receive
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
          if (serial_buffer.length() > 0) {  // ignore empty lines
              float commanded_power = serial_buffer.toFloat();
              motor_set_speed(&motor_LF, commanded_power);
          }
          serial_buffer = "";
      } else {
          serial_buffer += c;
    }
}

    unsigned long current_time = millis();
    if (current_time - last_print_time >= CONTROL_INTERVAL_MS) {
        last_print_time = current_time;

        as5600_get_velocity(&enc_LF);

        // Format: Time_ms, Power_pct, Angle_rad, Vel_rads
        Serial.print(millis());
        Serial.print(",");
        Serial.print(motor_LF.current_power, 2);
        Serial.print(",");
        Serial.print(enc_LF.radians, 6);
        Serial.print(",");
        Serial.println(enc_LF.angular_velocity, 6);
    }
}