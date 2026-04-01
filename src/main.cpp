#include <Arduino.h>
#include <Wire.h>
#include "imu.h"

// Constatns for Timing
const unsigned long INTERVAL_MS = 10; //100Hz
unsigned long last_print_time = 0;

// Structures
lsm6ds3_t imu;
sensor_data_t gyro_data;
sensor_data_t accel_data;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  while(!Serial);


  // Initialize IMU and I2C
  lsm6ds3_init(&imu,LSM6DS3_ADDR);

  // Authenticate IMU
  if (!lsm6ds3_check_id(&imu)) {
    Serial.println("IMU authentication failed!");
    while(1){ // Fast blink on failure
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    } // halt if authentication fails
  }
  Serial.println("IMU authentication successful!");

  // Enable IMU
  lsm6ds3_enable_accel(&imu);
  lsm6ds3_enable_gyro(&imu);

  // Calibrate IMU
  // Keep the IMU still
  uint32_t start_calibration_time = millis();
  uint32_t end_calibration_time = start_calibration_time + 10000;

  while (millis() <= end_calibration_time) {
    // Calibrate IMU and toggle LED to indicate calibration
    lsm6ds3_calibrate_accel(&imu);
    lsm6ds3_calibrate_gyro(&imu);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  Serial.println("IMU calibration complete!");

  delay(1000);
}

void loop() {

  unsigned long current_time = millis();
  if (current_time - last_print_time >= INTERVAL_MS) {
    last_print_time = current_time;
    // Read IMU data
    lsm6ds3_read_accel(&imu, &accel_data);
    lsm6ds3_read_gyro(&imu, &gyro_data);

    // Stream to MATLAB in CSV format
    Serial.print(accel_data.x);Serial.print(",");
    Serial.print(accel_data.y);Serial.print(",");
    Serial.print(accel_data.z);Serial.print(",");
    Serial.print(gyro_data.x);Serial.print(",");
    Serial.print(gyro_data.y);Serial.print(",");
    Serial.println(gyro_data.z);
  }  
}