/* ============================================================================
 * post.cpp - Power-On Self-Test (POST) & Hardware Integrity Framework
 * ============================================================================
 * Description:
 * Modular hardware validation framework executed at boot before the robot
 * enters its control loop. Performs structured I2C bus presence checks,
 * encoder magnetic health validation, and system integrity reporting.
 * If any critical check fails, the system halts with an SOS blink pattern
 * rather than attempting to run with faulty hardware.
 *
 * Hardware Target:
 * - Arduino Nano 33 IoT
 * - IMU:      LSM6DS3  (I2C @ 0x6A)
 * - MUX:      TCA9548A (I2C @ 0x70)
 * - Encoders: AS5600 × 4 (I2C @ 0x36, isolated per MUX channel)
 * - Bus Speed: 400kHz Fast Mode (set in post_init())
 *
 * Design Pattern:
 * - "Safety First" bootloader — hardware validated before any driver
 *   initialisation or control loop entry.
 * - Modular check functions return bool and print pass/fail to Serial,
 *   allowing granular diagnosis without a logic analyser.
 * - System halts on failure with specific diagnostic message rather than
 *   entering an undefined state with faulty sensor data.
 *
 * Check Sequence:
 * 1. post_init()          — configure I2C bus at 400kHz
 * 2. check_i2c()          — run all hardware checks, return overall health
 *    ├── post_check_main_bus(IMU)     — verify LSM6DS3 acknowledges on bus
 *    └── post_as5600_health(CH_LF)   — verify magnet present and signal strong
 * 3. post_system_report() — print result, halt on failure
 *
 * AS5600 Magnetic Health:
 * - Status register (0x0B) bit 5 (MD) must be 1 — magnet detected.
 * - AGC register (0x1A) value < 200 — signal strength sufficient.
 *   AGC near 255 indicates magnet too far or too weak.
 * - Distinguishes between NO MAGNET and WEAK SIGNAL for faster diagnosis.
 *
 * Extension:
 * - Additional encoder channels (LB, RF, RB) are commented out pending
 *   4-wheel hardware integration. Uncomment in check_i2c() as each
 *   encoder is physically installed and verified.
 *
 * Dependencies:
 * - Wire.h
 * - post.h
 * - as5600.h  (AS5600_ADDR, AS5600_STATUS_REG, AS5600_GAIN_CONTROL_REG)
 * - imu.h     (LSM6DS3_ADDR)
 * - config.h  (MUX_ADDR, MUX_CH_*, I2C_CLOCK)
 *
 * Author: Aminadab Z. Ghebrehiwet
 * Date:   2026-04-02
 * ============================================================================ */

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "as5600.h" 
#include "imu.h"
#include "post.h"   
#include "config.h"


bool system_health = true;
// Initialize POST
void post_init(void) {
    Wire.begin();
    Wire.setClock(I2C_CLOCK);
}

// Select MUX Channel
static void mux_select(uint8_t mux_addr, uint8_t mux_channel) {
    Wire.beginTransmission(mux_addr);
    Wire.write(1 << mux_channel);
    Wire.endTransmission();
}

// Check Main Bus
bool post_check_main_bus(uint8_t addr, const char* label) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    bool success = (error == 0);
    Serial.print(success?"[OK] ":"[FAIL] ");
    Serial.print(label);
    return success;
}

// Check MUX Channel (General Purpose Tool)
bool post_mux_channels(uint8_t mux_addr, uint8_t mux_channel, uint8_t device_addr, const char* label) {
    mux_select(mux_addr, mux_channel);

    Wire.beginTransmission(device_addr);
    uint8_t error = Wire.endTransmission();

    bool success = (error == 0);
    Serial.print(success?"[OK] ":"[FAIL] ");
    Serial.print(label);
    Serial.print("(MUX CH ");
    Serial.print(mux_channel);
    Serial.println(")");

    return success;
    
}

// Check AS5600 Encoder Health
bool post_as5600_health(uint8_t mux_channel, const char* label) {
    mux_select(MUX_ADDR, mux_channel);
    
    // Read AS5600 Status and Gain Control registers
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_STATUS_REG);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 1);
    uint8_t status = Wire.available() ? Wire.read() : 0x00;

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_GAIN_CONTROL_REG);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 1);
    uint8_t gain = Wire.available() ? Wire.read() : 0x80;

    // Logic: MD bit (bit 5) must be 1. 
    // MH/ML (bits 3/4) indicate air-gap issues.
    bool magnet_ok = (status & 0x20); // 0x20 is the "MD" bit
    bool gain_ok = (gain < 200);      // If AGC is near 255, magnet is too weak

    bool success = (magnet_ok && gain_ok);

    Serial.print(success ? "[ OK ] " : "[FAIL] ");
    Serial.print(label);
    if (!magnet_ok) Serial.print(" - NO MAGNET");
    else if (!gain_ok) Serial.print(" - WEAK SIGNAL");
    Serial.println(); 
    
    return success;
}

// System Integrity Check
bool post_system_report(bool health) {
    if (!health) {
        Serial.println("[FAIL] SYSTEM INTEGRITY CHECK");
        pinMode(LED_BUILTIN, OUTPUT);
        while (1){
            digitalWrite(LED_BUILTIN, HIGH);delay(100);
            digitalWrite(LED_BUILTIN, LOW);delay(100);
        }
        
    }
    Serial.println("[OK] SYSTEM INTEGRITY CHECK");
    Serial.println(F("Commencing boot...\n"));
    return health;
}

// Overall I2C Communicatio Check
bool check_i2c(void) {
    bool current_health = true; // Local tracking

    // 1. Verify IMU is physically on the bus
    current_health &= post_check_main_bus(LSM6DS3_ADDR, "IMU Hardware");

    // 2. Check Encoders + Magnetic Health
    current_health &= post_as5600_health(MUX_CH_LF, "Enc Left-Front");
    // current_health &= post_as5600_health(MUX_CH_LB, "Enc Left-Back");
    // current_health &= post_as5600_health(MUX_CH_RF, "Enc Right-Front");
    // current_health &= post_as5600_health(MUX_CH_RB, "Enc Right-Back");

    return current_health;
}






