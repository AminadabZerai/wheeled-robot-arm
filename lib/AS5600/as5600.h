/* ============================================================================
 * as5600.h - AS5600 Magnetic Rotary Encoder Driver Interface
 * ============================================================================
 * Exposes the public API for the AS5600 encoder driver. Defines the
 * as5600_t state struct, register addresses, configuration constants,
 * and function prototypes used by main.cpp and any other module that
 * needs encoder position or velocity data.
 *
 * Target:   Arduino Nano 33 IoT
 * Sensor:   AS5600 12-bit magnetic rotary encoder
 * Protocol: I2C via TCA9548A MUX (one encoder per channel)
 * Voltage:  3.3V logic
 *
 * Register Map (key entries):
 *   0x07–0x08  CONF       — configuration (hysteresis, power mode, filter)
 *   0x0B       STATUS     — magnet detection flags (MD, MH, ML)
 *   0x0C–0x0D  RAW_ANGLE  — 12-bit raw angle (0–4095 per revolution)
 *   0x1A       AGC        — automatic gain control (signal strength proxy)
 *
 * Preferred Configuration (AS5600_PREFERED_CONFIG = 0x40C):
 *   Power Mode:   Nominal (00)
 *   Hysteresis:   3 LSB   (11) — reduces noise in velocity derivation
 *   Output Stage: Analog  (00)
 *   Slow Filter:  1x      (01)
 *   Fast Filter:  6 LSB   (001)
 *   Watchdog:     Off     (0)
 *
 * Usage:
 *   as5600_t enc_LF;
 *   as5600_init(&enc_LF, MUX_CH_LF);
 *   as5600_get_velocity(&enc_LF);
 *   float vel = enc_LF.angular_velocity;  // rad/s
 *   float pos = enc_LF.radians;           // cumulative radians
 *
 * ============================================================================ */
 
#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* -- Configuration Registers -- */
#define AS5600_CONFIG_REG 0x07  // 2 bytes (0x07, 0x08)

/* -- Output Registers -- */
#define AS5600_RAW_ANGLE_REG 0x0C // 2 bytes (0x0C, 0x0D)
//#define AS6000_ANGLE_REG 0x0E  // Not used

/* -- Status Registers (also available in config.h for convenience) -- */
#define AS5600_STATUS_REG 0x0B
#define AS5600_GAIN_CONTROL_REG 0x1A

/* -- Automatic Gain Control Values -- */
// Used with 3.3v supply
// range between 0-128
#define AS5600_AUTO_GAIN_0 0x40 // 0.5 for most applications

/* AS5600 Customization Configuration */

//  * Nominal Power Mode: 00
//  * Hysteresis: 11 (3 LSB)
//  * Output Stage: 00 (Analog Full Range)
//  * PWMF: 00
//  * SF: 01 (1x)
//  * FTH: 001 (6 LSB)
//  * WD: 0 (Off)
//  * Binary: 0000 0100 0000 1100 -> 0x40C


#define AS5600_PREFERED_CONFIG 0x40C

// Structure to hold AS5600 configuration data
typedef struct {
    // Hardware connection
    uint8_t mux_channel;

    // Raw data and Roll Over Handling (0-4095)
    uint16_t last_raw_angle; // previous raw angle reading from register 0x0C
    int32_t total_ticks; // cumulative displacement to handle infinite rotation

    // Converted Physical Quantities
    float radians;  // Absolute postition for PID
    float delta_radians; // Change in radians
    float angular_velocity; // rad/s calculated from raw data during DT

} as5600_t;

/* Function Prototypes */

// Initialization
void as5600_init(as5600_t *sensor, uint8_t mux_channel);

// Data Acquisition
void as5600_get_angle(as5600_t *sensor);
void as5600_get_velocity(as5600_t *sensor);
void as5600_update(as5600_t *sensor);


#endif








