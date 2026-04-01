/* ============================================================================
 * imu.c - LSM6DS3 Hardware Abstraction Layer
 * ============================================================================
 * Description: 
 * Implementation of procedural driver for the onboard LSM6DS3 IMU. 
 * Handles I2C communication, configuration, calibration, and 
 * data normalization for robot navigation.
 * * Hardware Target:
 * - Arduino Nano 33 IoT (LSM6DS3)
 * - Bus: I2C (Wire library)
 * * Design Pattern:
 * - Procedural interface using (lsm6ds3_t *sensor) structures.
 * - Implements Burst-Read (Multi-byte) for data synchronization.
 * - Includes 16-bit signed integer conversion and gravity compensation.
 * * Dependencies:
 * - Wire.h
 * - imu.h
 * * Author: Aminadab Z. Ghebrehiwet
 * Date: 2026-04-01
 * ============================================================================ */

#include "imu.h"
#include <Arduino.h>
#include <math.h>
#include <Wire.h>

/* Helper Functions */

    // Function to write a byte to a register
static void write_reg(lsm6ds3_t *sensor,uint8_t reg, uint8_t value) {
    Wire.beginTransmission(sensor->addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

    // Function to read multiple bytes starting from a register
static void read_regs(lsm6ds3_t *sensor, uint8_t reg, uint8_t *buffer, uint8_t len) {
    Wire.beginTransmission(sensor->addr);
    Wire.write(reg);
    Wire.endTransmission(false); // false = repeated start, line locked until all bytes are read
    Wire.requestFrom(sensor->addr, len);
    for (int i = 0; i < len; ++i) 
    {
        if (Wire.available()) {
            buffer[i] = Wire.read();
        }
    }
}


/* IMU Functions */
    // Initialize IMU
void lsm6ds3_init(lsm6ds3_t *sensor, uint8_t addr) {
    sensor->addr = addr;
    Wire.begin();
}

    // Check IMU ID
bool lsm6ds3_check_id(lsm6ds3_t *sensor) {
    uint8_t id;
    read_regs(sensor, LSM6DS3_WHO_AM_I_REG, &id, 1);
    return id == LSM6DS3_CHIP_ID;
}

    // Enable Accelerometer: 104Hz, +/- 2g, 100Hz Filter
void lsm6ds3_enable_accel(lsm6ds3_t *sensor) {
    // 0x40 = 104Hz ODR
    // 0x00 = +/- 2g FS
    // 0x02 = 100Hz BW
    uint8_t config = 0x40 | 0x00 | 0x02; 
    write_reg(sensor, LSM6DS3_CTRL1_XL, config);
}

// Enable Gyroscope: 104Hz, +/- 125 dps
void lsm6ds3_enable_gyro(lsm6ds3_t *sensor) {
    // 0x40 = 104Hz ODR
    // 0x02 = This sets the FS_125 bit to 1
    uint8_t val = 0x40 | 0x02; 
    write_reg(sensor, LSM6DS3_CTRL2_G, val);
}  


/* IMU Data Acquisition Functions */

    // Read Accelerometer
void lsm6ds3_read_accel(lsm6ds3_t *sensor, sensor_data_t *data) {
    uint8_t buffer[6];
    
    // 1. Read all 6 bytes (XL_X_L, XL_X_H, XL_Y_L, XL_Y_H, XL_Z_L, XL_Z_H)
    read_regs(sensor, LSM6DS3_OUTX_L_XL, buffer, 6);

    //Combine bytes correctly: (High << 8) | Low (Little Endian)
    // Use (int16_t) to handle negative values
    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);

    // Apply calibration offsets
    data->x = (float)raw_x - sensor->offset_x_accel;
    data->y = (float)raw_y - sensor->offset_y_accel;
    data->z = (float)raw_z - sensor->offset_z_accel;
}
    // Calibrate Accelerometer
void lsm6ds3_calibrate_accel(lsm6ds3_t *sensor) {

    uint8_t buffer[6];
    // read all 6 bytes starting from XL_X_L

    int32_t sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < CALIBRATION_DATA_SIZE; i++) {
        
        read_regs(sensor, LSM6DS3_OUTX_L_XL, buffer, 6);


        
        //Combine bytes correctly: (High << 8) | Low (Little Endian)
        int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
        int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
        int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);

        sum_x += raw_x;
        sum_y += raw_y;
        sum_z += raw_z;

        delay(2); //wait 2ms to avoid overloading for sensor to refresh
    }


    sensor->offset_x_accel = sum_x / CALIBRATION_DATA_SIZE;
    sensor->offset_y_accel = sum_y / CALIBRATION_DATA_SIZE;
    sensor->offset_z_accel = (sum_z / CALIBRATION_DATA_SIZE) - GRAVITY_WHEN_STATIC;
}

 // Read Gyroscope
void lsm6ds3_read_gyro(lsm6ds3_t *sensor, sensor_data_t *data) {
    uint8_t buffer[6];
    
    // 1. Read all 6 bytes starting from X_L_G
    read_regs(sensor, LSM6DS3_OUTX_L_G, buffer, 6);

    //Combine bytes correctly: (High << 8) | Low (Little Endian)
    // Use (int16_t) to handle negative values
    int16_t raw_gx = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_gy = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_gz = (int16_t)((buffer[5] << 8) | buffer[4]);

    // Apply calibration offsets
    data->x = (float)raw_gx - sensor->offset_x_gyro;
    data->y = (float)raw_gy - sensor->offset_y_gyro;
    data->z = (float)raw_gz - sensor->offset_z_gyro;
}
    // Calibrate Accelerometer
void lsm6ds3_calibrate_gyro(lsm6ds3_t *sensor) {

    uint8_t buffer[6];
    // read all 6 bytes starting from X_L_G

    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < CALIBRATION_DATA_SIZE; i++) {
        
        read_regs(sensor, LSM6DS3_OUTX_L_G, buffer, 6);


        
        //Combine bytes correctly: (High << 8) | Low (Little Endian)
        int16_t raw_gx = (int16_t)((buffer[1] << 8) | buffer[0]);
        int16_t raw_gy = (int16_t)((buffer[3] << 8) | buffer[2]);
        int16_t raw_gz = (int16_t)((buffer[5] << 8) | buffer[4]);

        sum_gx += raw_gx;
        sum_gy += raw_gy;
        sum_gz += raw_gz;

        delay(2); //wait 2ms to avoid overloading for sensor to refresh
    }


    sensor->offset_x_gyro = sum_gx / CALIBRATION_DATA_SIZE;
    sensor->offset_y_gyro = sum_gy / CALIBRATION_DATA_SIZE;
    sensor->offset_z_gyro = (sum_gz / CALIBRATION_DATA_SIZE);
}





