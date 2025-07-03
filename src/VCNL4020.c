/*
C driver for VCNL4020 Proximity and Ambient Light Sensor
This driver provides functions to initialize the sensor, set the IR LED current,
set the sensor mode, and read proximity and ambient light values.
It is designed to be used with Arduino and compatible platforms.

Author: ARI ESTERS
Date: 3 JULY 2025
License: MIT License 

** Coming soon: 
 - I2C commands 
- Platform portability for I2C logic
- Additional functions for sensor configuration (interrupts, change measurement rates, etc)
*/

#include "VCNL4020.h"
#include <stdint.h>

// Initialize sensor in idle state
VCNL4020 VCNL4020_Init(uint8_t i2c_address) 
{
    // Set the sensor's initial configuration
    VCNL4020 sensor;
    sensor.address = i2c_address;
    sensor.current = 0; 
    sensor.mode = DISABLED; 

    /*
    *  Confirm I2C comms with device
    */
}

// Set IR LED current for proximity measurements (does not turn on/off the LED)
void VCNL4020_SetCurrent(VCNL4020 *sensor, uint8_t led_current) 
{
    /*
    * Check if current is within valid range
    * Write "current" value divided by 10 to proper register via I2C
    * Update "current" field in the "sensor" struct
    */
}

void VCNL4020_SetMode(VCNL4020 *sensor, VCNL4020_Mode mode) 
{
    // Set the sensor's mode
    sensor->mode = mode;

    /*
    * Check if the mode is valid
    * Write "mode" value to the proper register via I2C
    * Update the "mode" field in the "sensor" struct
    */ 
}

uint16_t VCNL4020_ReadProximity(VCNL4020 *sensor) 
{
    // Read proximity data from the sensor
    // TBD: Specific I2C implementation

    return -1; // Return -1 if no data available
}

uint16_t VCNL4020_ReadAmbientLight(VCNL4020 *sensor) 
{
    // Read ambient light data from the sensor
    // TBD: Specific I2C implementation

    return -1; // Return -1 if no data available
}