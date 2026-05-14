/**
 * @file drv8214_platform_i2c.cpp
 * @brief Platform-specific I2C implementations for the DRV8214 library.
 *
 * @copyright Copyright (c) 2026 Theo Heng
 * @license MIT License. See LICENSE file for details.
 */

#include "drv8214_platform_i2c.h"

bool drv8214_i2c_write_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t value) {
    if (bus == NULL) {
        return false;
    }

#ifdef DRV8214_PLATFORM_ARDUINO
    bus->beginTransmission(device_address);
    bus->write(reg);
    bus->write(value);
    return bus->endTransmission() == 0;
#elif defined(DRV8214_PLATFORM_STM32)
    uint8_t data[2] = { reg, value };
    return HAL_I2C_Master_Transmit(bus, (uint16_t)(device_address << 1), data, 2, HAL_MAX_DELAY) == HAL_OK;
#elif defined(DRV8214_PLATFORM_RP2040)
    uint8_t data[2] = { reg, value };
    return i2c_write_blocking(bus, device_address, data, 2, false) == 2;
#else
    (void)device_address;
    (void)reg;
    (void)value;
    return false;
#endif
}

bool drv8214_i2c_read_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t* out) {
    if (bus == NULL || out == NULL) {
        return false;
    }

#ifdef DRV8214_PLATFORM_ARDUINO
    bus->beginTransmission(device_address);
    bus->write(reg);
    uint8_t result = bus->endTransmission(false);
    if (result != 0) {
        return false;
    }
    if (bus->requestFrom(device_address, static_cast<uint8_t>(1)) != 1) {
        return false;
    }
    if (!bus->available()) {
        return false;
    }
    *out = bus->read();
    return true;
#elif defined(DRV8214_PLATFORM_STM32)
    if (HAL_I2C_Master_Transmit(bus, (uint16_t)(device_address << 1),
                                &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }
    return HAL_I2C_Master_Receive(bus, (uint16_t)(device_address << 1),
                                  out, 1, HAL_MAX_DELAY) == HAL_OK;
#elif defined(DRV8214_PLATFORM_RP2040)
    if (i2c_write_blocking(bus, device_address, &reg, 1, true) != 1) {
        return false;
    }
    return i2c_read_blocking(bus, device_address, out, 1, false) == 1;
#else
    (void)device_address;
    (void)reg;
    return false;
#endif
}

bool drv8214_i2c_modify_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t enable_bits) {
    uint8_t current_value = 0;
    if (!drv8214_i2c_read_register(bus, device_address, reg, &current_value)) {
        return false;
    }
    if (enable_bits) {
        current_value |= mask;
    } else {
        current_value &= ~mask;
    }
    return drv8214_i2c_write_register(bus, device_address, reg, current_value);
}

bool drv8214_i2c_modify_register_bits(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value) {
    uint8_t current_value = 0;
    if (!drv8214_i2c_read_register(bus, device_address, reg, &current_value)) {
        return false;
    }
    current_value = (current_value & ~mask) | (new_value & mask);
    return drv8214_i2c_write_register(bus, device_address, reg, current_value);
}
