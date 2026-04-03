#include <Arduino.h>
#include <Wire.h>
#include "as5600.h"
#include "imu.h"
#include "config.h"
#include "post.h"


// Constants for Timing
unsigned long last_print_time = 0;

// Structures
lsm6ds3_t imu;
as5600_t enc_LF;
sensor_data_t gyro_data;
sensor_data_t accel_data;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while(!Serial);

  delay(5000); // Wait 5 seconds just for debug

  post_init(); // Sets 400kHz clock

  // Step 1: Physical I2C Check
  bool health = check_i2c();
  post_system_report(health); // Halts here if a wire is loose

  // Step 2: Initialize IMU Object and Encoder Object
  lsm6ds3_init(&imu, LSM6DS3_ADDR);
  as5600_init(&enc_LF, MUX_CH_LF);

  // Step 3: Authenticate (Verify it's actually an LSM6DS3)
  if (!lsm6ds3_check_id(&imu)) {
    Serial.println(F("IMU Authentication Failed! Wrong WHO_AM_I value."));
    while(1) { // SOS Blink
      digitalWrite(LED_BUILTIN, HIGH); delay(50);
      digitalWrite(LED_BUILTIN, LOW);  delay(50);
    }
  }
  Serial.println(F("IMU Authentication Successful!"));

  // Step 4: Enable & Calibrate
  lsm6ds3_enable_accel(&imu);
  lsm6ds3_enable_gyro(&imu);

  Serial.println(F("Calibrating... Keep Robot Still."));
  uint32_t end_cal = millis() + 10000;
  while (millis() <= end_cal) {
    // Visual feedback: Keep LED ON
    digitalWrite(LED_BUILTIN, HIGH);
    lsm6ds3_calibrate_accel(&imu);
    lsm6ds3_calibrate_gyro(&imu);   
 }
  Serial.println(F("Calibration Complete. Ready."));
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {

  unsigned long current_time = millis();
  if (current_time - last_print_time >= CONTROL_INTERVAL_MS) {
    last_print_time = current_time;
    // Read IMU data
    lsm6ds3_read_accel(&imu, &accel_data);
    lsm6ds3_read_gyro(&imu, &gyro_data);

    //Update Encoder (updates total_ticks, radians, and velocity)
    as5600_get_velocity(&enc_LF);



    // Stream to MATLAB in CSV format
    Serial.print(accel_data.x);Serial.print(",");
    Serial.print(accel_data.y);Serial.print(",");
    Serial.print(accel_data.z);Serial.print(",");
    Serial.print(gyro_data.x);Serial.print(",");
    Serial.print(gyro_data.y);Serial.print(",");
    Serial.print(gyro_data.z);Serial.print(",");
    Serial.print(enc_LF.radians);Serial.print(",");
    Serial.println(enc_LF.angular_velocity);
  } 
}