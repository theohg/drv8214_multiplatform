/**
 * @file drv8214_platform_i2c.cpp
 * @brief Platform-specific I2C implementations for the DRV8214 library.
 *
 * @copyright Copyright (c) 2025 Theo Heng
 * @license MIT License. See LICENSE file for details.
 */

#include "drv8214_platform_i2c.h"

#ifdef DRV8214_PLATFORM_STM32
static I2C_HandleTypeDef* drv_i2c_handle = NULL;

void drv8214_i2c_set_handle(I2C_HandleTypeDef* hi2c) {
    drv_i2c_handle = hi2c;
}
#endif

void drv8214_i2c_write_register(uint8_t device_address, uint8_t reg, uint8_t value) {
#ifdef DRV8214_PLATFORM_ARDUINO
    Wire.beginTransmission(device_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
#elif defined(DRV8214_PLATFORM_STM32)
    if (drv_i2c_handle == NULL) { return; }
    uint8_t data[2] = { reg, value };
    HAL_I2C_Master_Transmit(drv_i2c_handle, (uint16_t)(device_address << 1), data, 2, HAL_MAX_DELAY);
#endif
}

uint8_t drv8214_i2c_read_register(uint8_t device_address, uint8_t reg) {
#ifdef DRV8214_PLATFORM_ARDUINO
    Wire.beginTransmission(device_address);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(device_address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0;
#elif defined(DRV8214_PLATFORM_STM32)
    if (drv_i2c_handle == NULL) { return 0; }
    uint8_t data = 0;
    if (HAL_I2C_Master_Transmit(drv_i2c_handle, (uint16_t)(device_address << 1), &reg, 1, HAL_MAX_DELAY) == HAL_OK) {
        HAL_I2C_Master_Receive(drv_i2c_handle, (uint16_t)(device_address << 1), &data, 1, HAL_MAX_DELAY);
    }
    return data;
#else
    return 0;
#endif
}

void drv8214_i2c_modify_register(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t enable_bits) {
    uint8_t current_value = drv8214_i2c_read_register(device_address, reg);
    if (enable_bits) {
        current_value |= mask;
    } else {
        current_value &= ~mask;
    }
    drv8214_i2c_write_register(device_address, reg, current_value);
}

void drv8214_i2c_modify_register_bits(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value) {
    uint8_t current_value = drv8214_i2c_read_register(device_address, reg);
    current_value = (current_value & ~mask) | (new_value & mask);
    drv8214_i2c_write_register(device_address, reg, current_value);
}
