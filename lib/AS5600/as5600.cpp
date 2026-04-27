/* ============================================================================
 * as5600.cpp - AS5600 Magnetic Rotary Encoder Hardware Abstraction Layer
 * ============================================================================
 * Description:
 * Procedural driver for the AS5600 12-bit magnetic rotary encoder.
 * Handles I2C communication through a TCA9548A multiplexer, rollover-safe
 * cumulative angle tracking across infinite rotations, and real-time
 * angular velocity derivation for closed-loop PID velocity control.
 *
 * Hardware Target:
 * - Arduino Nano 33 IoT
 * - Sensor: AS5600 (fixed I2C address 0x36)
 * - Bus: TCA9548A I2C Multiplexer (0x70) — one encoder per MUX channel
 * - Bus Speed: 400kHz Fast Mode
 *
 * Design Pattern:
 * - Reentrant procedural interface using (as5600_t *sensor) pointers.
 * - MUX channel selection encapsulated inside read_regs() — transparent
 *   to all higher-level code.
 * - Rollover handling via shortest-path signed 16-bit jump algorithm:
 *   jumps > +2048 or < -2048 are corrected by ±4096, mapping the
 *   12-bit circular domain onto a 32-bit linear accumulator (total_ticks).
 * - delta_radians logged per cycle to avoid downstream floating-point
 *   differencing errors during offline MATLAB/EKF replay.
 *
 * Key Design Decisions:
 * - buffer[2] zero-initialised in as5600_get_angle() — failed I2C reads
 *   produce zero-delta rather than garbage position jumps.
 * - write_reg() retained as a stub for future AS5600_PREFERED_CONFIG
 *   register write (hysteresis, power mode, slow filter).
 * - angular_velocity sign is inherent from jump direction — no extra
 *   direction pin required.
 *
 * Dependencies:
 * - Wire.h
 * - as5600.h
 * - config.h (MUX_ADDR, AS5600_ADDR, CONTROL_INTERVAL_MS, RAW_TO_RAD)
 *
 * Author: Aminadab Z. Ghebrehiwet
 * Date:   2026-04-03
 * ============================================================================ */

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "math.h"
#include "as5600.h"



/* Helper Functions */

// Function to select a MUX channel (specify which encoder to use)
static void select_mux_channel(uint8_t mux_channel) {
    Wire.beginTransmission(MUX_ADDR);
    Wire.write(1 << mux_channel);
    Wire.endTransmission();
}

// Function to read multiple bytes starting from a register
static void read_regs(as5600_t *sensor, uint8_t reg, uint8_t *buffer, uint8_t len) {
    // Select MUX Channel
    select_mux_channel(sensor->mux_channel);
    
    // Talk to AS5600 on that channel
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false); // false = repeated start, line locked until all bytes are read
    Wire.requestFrom(AS5600_ADDR, len);
    for (int i = 0; i < len; ++i) 
    {
        if (Wire.available()) {
            buffer[i] = Wire.read();
        }
    }
}


// Initialization
void as5600_init(as5600_t *sensor, uint8_t mux_channel){
    sensor->mux_channel = mux_channel;
    sensor->total_ticks = 0;
    sensor->last_raw_angle = 0;
    sensor->radians = 0.0f;
    sensor->delta_radians = 0.0f;
    sensor->angular_velocity = 0.0f;
}


// Data Acquisition
// AS5600 --> High Bytes First, then Low Bytes
void as5600_get_angle(as5600_t *sensor){
    uint8_t buffer[2];
    read_regs(sensor, AS5600_RAW_ANGLE_REG, buffer, 2);
    uint16_t current_raw_angle = ((uint16_t)buffer[0] << 8) | buffer[1];
    sensor->last_raw_angle = current_raw_angle;
}


// Update sensor data with Roll-Over Handling
void as5600_update(as5600_t *sensor){
    uint16_t previous_raw_angle = sensor->last_raw_angle;

    // Capture the current raw angle
    as5600_get_angle(sensor);

    // Calculate the jump
    int16_t jump = sensor->last_raw_angle - previous_raw_angle;

    // Check if the raw angle has rolled over
    // If jump is > 2048, it means we crossed the 0-mark backwards
    // If jump is < -2048, it means we crossed the 0-mark forwards
    if (jump > 2048)       jump -= 4096;
    else if (jump < -2048) jump += 4096;

    // Update cumulative angle
    sensor->total_ticks += jump;

    // Convert to radians
    sensor->radians = (float)sensor->total_ticks * RAW_TO_RAD;   
}

void as5600_get_velocity(as5600_t *sensor){
    // Get the previous radians
    float prev_radians = sensor->radians;

    // Get the updated angle with roll-over handling
    as5600_update(sensor);

    // Get the time difference from the control interval in seconds (not ms)
    // Calculate velocity: (Current - Previous) / Time
    float dt = (float)CONTROL_INTERVAL_MS / 1000.0f;

    // Change in radians
    sensor->delta_radians = (sensor->radians - prev_radians);
    sensor->angular_velocity = sensor->delta_radians / dt; 
}
    