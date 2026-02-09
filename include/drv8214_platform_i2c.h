/**
 * @file drv8214_platform_i2c.h
 * @brief Platform-abstracted I2C interface for the DRV8214 library.
 *
 * Provides a common I2C API used by the DRV8214 driver class. The Arduino
 * implementation uses the Wire library; the STM32 implementation uses HAL.
 *
 * @note On STM32, call drv8214_i2c_set_handle() once before any I2C operation.
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

#ifdef DRV8214_PLATFORM_STM32
    /**
     * @brief Set the HAL I2C handle used by all DRV8214 I2C operations.
     * @param hi2c Pointer to an initialized I2C_HandleTypeDef (e.g. &hi2c1).
     */
    void drv8214_i2c_set_handle(I2C_HandleTypeDef* hi2c);
#endif

/* ──────────────────── I2C Error Codes ──────────────────── */

/** @brief Success return code. */
#define DRV8214_OK          0
/** @brief I2C communication error (NACK, timeout, bus error). */
#define DRV8214_ERR_I2C     1
/** @brief STM32: I2C handle not initialized (call drv8214_i2c_set_handle first). */
#define DRV8214_ERR_HANDLE  2

/**
 * @brief Write a single byte to a device register.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @param value Byte to write.
 * @return DRV8214_OK on success, DRV8214_ERR_I2C or DRV8214_ERR_HANDLE on failure.
 */
uint8_t drv8214_i2c_write_register(uint8_t device_address, uint8_t reg, uint8_t value);

/**
 * @brief Read a single byte from a device register.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @return The byte read, or 0 on failure (check drv8214_i2c_get_last_error()).
 */
uint8_t drv8214_i2c_read_register(uint8_t device_address, uint8_t reg);

/**
 * @brief Get the error code from the last I2C operation.
 * @return DRV8214_OK if the last operation succeeded, or an error code.
 */
uint8_t drv8214_i2c_get_last_error(void);

/**
 * @brief Set or clear specific bits in a register using a mask.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @param mask Bitmask selecting which bits to modify.
 * @param enable_bits Non-zero to set the masked bits, zero to clear them.
 * @return DRV8214_OK on success, or an error code on failure.
 */
uint8_t drv8214_i2c_modify_register(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t enable_bits);

/**
 * @brief Replace specific bits in a register with a new value.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @param mask Bitmask selecting which bits to replace.
 * @param new_value New bit values (only masked bits are applied).
 * @return DRV8214_OK on success, or an error code on failure.
 */
uint8_t drv8214_i2c_modify_register_bits(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value);

#endif /* DRV8214_PLATFORM_I2C_H */
