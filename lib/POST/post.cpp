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






