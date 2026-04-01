/* ============================================================================
 * imu.h - LSM6DS3 Driver for Arduino Nano 33 IoT
 * ============================================================================
 * This library provides a procedural C interface to the onboard LSM6DS3 IMU.
 * It handles initialization, raw data retrieval, and gyroscope calibration based on the datasheet
 * * Target: Arduino Nano 33 IoT (LSM6DS3)
 * Logic Voltage: 3.3V
 * Protocol: I2C (Wire)
 * ============================================================================ */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <Wire.h>

/* -- Device Constants -- */
#define LSM6DS3_ADDR    0x6A  // Default I2C address for Arduino Nano 33 IoT LSM6DS3
#define LSM6DS3_WHO_AM_I_REG 0x0F // Device ID Register
#define LSM6DS3_CHIP_ID 0x69 // Chip ID for LSM6DS3

/* Sensitivity Constants */
#define LSM6DS3_SENSITIVITY_ACCEL 0.061f   // mg/LSB
#define LSM6DS3_SENSITIVITY_GYRO  0.004375f   // dps/LSB
#define CALIBRATION_DATA_SIZE 1000  // Use this many samples to calibrate

/* Control Registers */
#define LSM6DS3_CTRL1_XL 0x10  // Accelerometer control register
#define LSM6DS3_CTRL2_G  0x11  // Gyroscope control register

/* Data Output Registers */

        // Gyroscope
#define LSM6DS3_OUTX_L_G  0x22  // Gyroscope X low byte
#define LSM6DS3_OUTX_H_G  0x23  // Gyroscope X high byte
#define LSM6DS3_OUTY_L_G  0x24  // Gyroscope Y low byte
#define LSM6DS3_OUTY_H_G  0x25  // Gyroscope Y high byte
#define LSM6DS3_OUTZ_L_G  0x26  // Gyroscope Z low byte
#define LSM6DS3_OUTZ_H_G  0x27  // Gyroscope Z high byte

        // Accelerometer
#define LSM6DS3_OUTX_L_XL 0x28  // Accelerometer X low byte
#define LSM6DS3_OUTX_H_XL 0x29  // Accelerometer X high byte
#define LSM6DS3_OUTY_L_XL 0x2A  // Accelerometer Y low byte
#define LSM6DS3_OUTY_H_XL 0x2B  // Accelerometer Y high byte
#define LSM6DS3_OUTZ_L_XL 0x2C  // Accelerometer Z low byte
#define LSM6DS3_OUTZ_H_XL 0x2D  // Accelerometer Z high byte

/* Accelerometer ANTI-ALIASING FILTER BANDWIDTH Selection */
#define LSM6DS3_BW_400HZ  0x00
#define LSM6DS3_BW_200HZ  0x01
#define LSM6DS3_BW_100HZ  0x02
#define LSM6DS3_BW_50HZ   0x03

/* Accelerometer full-scale RANGE Selection */
#define LSM6DS3_FS_2G  0x00
#define LSM6DS3_FS_4G  0x01
#define LSM6DS3_FS_8G  0x02
#define LSM6DS3_FS_16G 0x03

// IMU Z-Axis Offset when statically level
#define GRAVITY_WHEN_STATIC 16384 // (2^15/2)


/* Simple structure to hold sensor data */
typedef struct {
    float x, y, z;
} sensor_data_t;  // Sensor data parameterized as *data in functions in imu.c

/* Driver structure */
typedef struct {
    uint8_t addr;       // I2C address
    bool is_ready;      // True if sensor is ready
    float offset_x_accel, offset_y_accel, offset_z_accel;  // Accelerometer calibration offsets
    float offset_x_gyro, offset_y_gyro, offset_z_gyro;     // Gyroscope calibration offsets
} lsm6ds3_t;    // LSM6DS3 driver parameterized as *sensor in functions in imu.c

/* Function Prototypes */

        // Initialization and Verifcation
void lsm6ds3_init(lsm6ds3_t *sensor, uint8_t addr);
bool lsm6ds3_check_id(lsm6ds3_t *sensor);

        // Accelerometer related functions
void lsm6ds3_enable_accel(lsm6ds3_t *sensor);
void lsm6ds3_read_accel(lsm6ds3_t *sensor, sensor_data_t *data);
void lsm6ds3_calibrate_accel(lsm6ds3_t *sensor);

        // Gyroscope related functions
void lsm6ds3_enable_gyro(lsm6ds3_t *sensor);
void lsm6ds3_read_gyro(lsm6ds3_t *sensor, sensor_data_t *data);
void lsm6ds3_calibrate_gyro(lsm6ds3_t *sensor);


#endif