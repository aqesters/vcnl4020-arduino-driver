#ifndef VCNL4020_H
#define VCNL4020_H

#include <stdint.h>
#include <Arduino.h>

// Registers in VCNL4020 proximity device
#define ADDRESS_REG00 0X80    // Command register 
#define ADDRESS_REG01 0x81    // Product revision register
//#define ADDRESS_REG01 0x82    // Proximity measurement rate register
#define ADDRESS_REG03 0x83    // IR LED current register
//#define ADDRESS_REG04 0x84    // Ambient light measurement rate register
#define ADDRESS_REG05 0x85    // Ambient light results upper byte
#define ADDRESS_REG07 0x87    // Proximity results upper byte
//#define ADDRESS_REG14 0x8E    // Read and clear interrupts (to clear all, write "0b1111")

/*
@brief VCNL4020 modes
@details
- DISABLED: Sensor is idle, no measurements taken
- MANUAL: Self-timed mode, measurements taken by explicit command
- AUTO_PROX: Enables periodic proximity measurements
- AUTO_ALS: Enables periodic ambient light sensing measurements
- FULL_AUTO: Periodically measure proximity and ambient light 
@note - If MANUAL mode is set, then readProximity() and readAmbientLight() perform a single measurement.
@note - If FULL_AUTO mode is set, then readProximity() and readAmbientLight() return the last measured value.
*/
typedef enum {
    DISABLED = 0,
    MANUAL = 1,
    AUTO_PROX = (1 << 1),
    AUTO_ALS = (1 << 2),
    FULL_AUTO = (AUTO_PROX | AUTO_ALS) // Both proximity and ambient light sensing enabled
} VCNL4020_Mode;

// struct for VCNL4020 sensor configuration
typedef struct 
{
    uint8_t address; // I2C address of the VCNL4020 sensor
    uint8_t current; // IR LED current setting
    VCNL4020_Mode mode;
} VCNL4020;

// Functions for sensor 
void VCNL4020_Init(VCNL4020 *sensor, uint8_t i2c_address);
void VCNL4020_SetCurrent(VCNL4020 *sensor, uint8_t current);
void VCNL4020_SetMode(VCNL4020 *sensor, VCNL4020_Mode mode);
uint16_t VCNL4020_ReadProximity(VCNL4020 *sensor);
uint16_t VCNL4020_ReadAmbientLight(VCNL4020 *sensor);
//uint8_t VCNL4020_SetProxMeasurementRate(VCNL4020 *sensor, uint8_t rate_factor);
//uint8_t VCNL4020_SetAmbientMeasurementRate(VCNL4020 *sensor, uint8_t rate_factor);
//uint8_t VCNL4020_SetInterrupts(VCNL4020 *sensor, ...);
//uint8_t VCNL4020_ClearInterrupts(VCNL4020 *sensor);
//uint8_t VCNL4020_InterruptStatus(VCNL4020 *sensor);

/* Coming soon: The above functions that are commented out! */

#endif // VCNL4020_H