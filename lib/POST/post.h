#ifndef POST_H
#define POST_H

#include <Arduino.h>
#include <Wire.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


// I2C Status codes
typedef enum {
    POST_OK = 0,
    POST_ERROR_BUSY = 1,
    POST_ERROR_DATA_NACK = 2,
    POST_ERROR_ADDR_NACK = 3,
    POST_ERROR_OTHER = 4
}post_status_t;


// Initialize POST (I2C and MUX)
void post_init(void);

// Hardware Pings
bool post_check_main_bus(uint8_t addr, const char* label);
bool post_mux_channels(uint8_t mux_addr, uint8_t mux_channel,uint8_t device_addr, const char* label);

// System Integrity Check
bool check_i2c(void);
// Diagnostics
bool post_as5600_health(uint8_t mux_channel, const char* label);
bool post_system_report(bool health);

#ifdef __cplusplus
}
#endif

#endif

