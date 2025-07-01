#include "VCNL4020.h"
#include <stdint.h>
#include <Arduino.h>

// Initialize sensor in idle state
void VCNL4020_Init(VCNL4020 *sensor, uint8_t i2c_address = "") 
{
    // Default address to 0x13 if not specified
    if (i2c_address == "") {
        i2c_address = 0x13; 
    } else {
        i2c_address = i2c_address; // Use the provided address
    }

    // Set the sensor's initial configuration
    sensor->address = i2c_address;
    sensor->current = 0;
    sensor->mode = DISABLED;

    // Initialize I2C communication
    Wire.begin();
    Wire.beginTransmission(sensor->address);
    Wire.write(ADDRESS_REG00); // Write to the control register
    Wire.write(sensor->mode); // Set sensor to idle mode
    Wire.endTransmission();
}

// Set IR LED current for proximity measurements (does not turn on/off the LED)
void VCNL4020_SetCurrent(VCNL4020 *sensor, uint8_t led_current) 
{
    // Check if current is within valid range
    // write current value to the sensor
    // update current value
}

void VCNL4020_SetMode(VCNL4020 *sensor, VCNL4020_Mode mode) 
{
    // Set the sensor's mode
    sensor->mode = mode;

    // Write the new mode to the sensor
    Wire.beginTransmission(sensor->address);
    Wire.write(ADDRESS_REG00);  // Control register address
    Wire.write(sensor->mode);   // Set the mode
    Wire.endTransmission();
}

uint16_t VCNL4020_ReadProximity(VCNL4020 *sensor) 
{
    // Read proximity data from the sensor
    Wire.beginTransmission(sensor->address);
    Wire.write(ADDRESS_REG07); // Proximity results upper byte address
    Wire.endTransmission();
    
    Wire.requestFrom(sensor->address, 2); // Request 2 bytes for proximity data
    if (Wire.available() == 2) {
        uint16_t prox_data = (Wire.read() << 8) | Wire.read(); // Combine two bytes
        return prox_data;
    }
    return -1; // Return -1 if no data available
}

uint16_t VCNL4020_ReadAmbientLight(VCNL4020 *sensor) 
{
    // Read ambient light data from the sensor
    Wire.beginTransmission(sensor->address);
    Wire.write(ADDRESS_REG05); // Ambient light results upper byte address
    Wire.endTransmission();
    
    Wire.requestFrom(sensor->address, 2); // Request 2 bytes for ambient light data
    if (Wire.available() == 2) {
        uint16_t als_data = (Wire.read() << 8) | Wire.read(); // Combine two bytes
        return als_data;
    }
    return -1; // Return -1 if no data available
}