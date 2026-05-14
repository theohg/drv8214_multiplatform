/**
 * @file drv8214_platform_i2c.h
 * @brief Platform-abstracted I2C interface for the DRV8214 library.
 *
 * Provides a common I2C API used by the DRV8214 driver class. The Arduino
 * implementation uses the Wire library; the STM32 implementation uses HAL;
 * the RP2040 implementation uses the Pico SDK.
 *
 * @note Device addresses are always passed as 7-bit addresses.
 *       STM32 HAL shifts them internally; Arduino Wire and Pico SDK expect 7-bit.
 *
 * @copyright Copyright (c) 2026 Theo Heng
 * @license MIT License. See LICENSE file for details.
 */

#ifndef DRV8214_PLATFORM_I2C_H
#define DRV8214_PLATFORM_I2C_H

#include "drv8214_platform_config.h"

#ifdef DRV8214_PLATFORM_ARDUINO
    #include <Wire.h>
#endif

#if defined(PLATFORM_ARDUINO)
    typedef TwoWire* bus_handle_t;
#elif defined(PLATFORM_STM32)
    typedef I2C_HandleTypeDef* bus_handle_t;
#elif defined(PLATFORM_RP2040)
    typedef i2c_inst_t* bus_handle_t;
#else
    typedef void* bus_handle_t;
#endif

/* ──────────────────── I2C Error Codes ──────────────────── */

/** @brief Success return code. */
#define DRV8214_OK          0
/** @brief I2C communication error (NACK, timeout, bus error). */
#define DRV8214_ERR_I2C     1
/** @brief Bus handle not initialized. */
#define DRV8214_ERR_HANDLE  2

/**
 * @brief Write a single byte to a device register.
 * @param bus Platform-specific I2C bus handle.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @param value Byte to write.
 * @return true on success, false on failure.
 */
bool drv8214_i2c_write_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t value);

/**
 * @brief Read a single byte from a device register.
 * @param bus Platform-specific I2C bus handle.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @param out Destination for the register value.
 * @return true on success, false on failure.
 */
bool drv8214_i2c_read_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t* out);

/**
 * @brief Set or clear specific bits in a register using a mask.
 * @param bus Platform-specific I2C bus handle.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @param mask Bitmask selecting which bits to modify.
 * @param enable_bits Non-zero to set the masked bits, zero to clear them.
 * @return true on success, false on failure.
 */
bool drv8214_i2c_modify_register(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t enable_bits);

/**
 * @brief Replace specific bits in a register with a new value.
 * @param bus Platform-specific I2C bus handle.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @param mask Bitmask selecting which bits to replace.
 * @param new_value New bit values (only masked bits are applied).
 * @return true on success, false on failure.
 */
bool drv8214_i2c_modify_register_bits(bus_handle_t bus, uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value);

#endif /* DRV8214_PLATFORM_I2C_H */
