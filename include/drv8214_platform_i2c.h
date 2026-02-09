/**
 * @file drv8214_platform_i2c.h
 * @brief Platform-abstracted I2C interface for the DRV8214 library.
 *
 * Provides a common I2C API used by the DRV8214 driver class. The Arduino
 * implementation uses the Wire library; the STM32 implementation uses HAL.
 *
 * @note On STM32, call drv8214_i2c_set_handle() once before any I2C operation.
 * @note On STM32, change the HAL include below to match your MCU family
 *       (e.g. stm32f4xx_hal.h for STM32F4).
 *
 * @copyright Copyright (c) 2025 Theo Heng
 * @license MIT License. See LICENSE file for details.
 */

#ifndef DRV8214_PLATFORM_I2C_H
#define DRV8214_PLATFORM_I2C_H

#include "drv8214_platform_config.h"

#ifdef DRV8214_PLATFORM_ARDUINO
    #include <Wire.h>
#endif

#ifdef DRV8214_PLATFORM_STM32
    /* Auto-detect STM32 family HAL header, or override via build flag:
       -DDRV8214_STM32_HAL_HEADER='"stm32f4xx_hal.h"'                    */
    #if defined(DRV8214_STM32_HAL_HEADER)
        #include DRV8214_STM32_HAL_HEADER
    #elif defined(STM32C0xx)
        #include "stm32c0xx_hal.h"
    #elif defined(STM32F0xx)
        #include "stm32f0xx_hal.h"
    #elif defined(STM32F1xx)
        #include "stm32f1xx_hal.h"
    #elif defined(STM32F2xx)
        #include "stm32f2xx_hal.h"
    #elif defined(STM32F3xx)
        #include "stm32f3xx_hal.h"
    #elif defined(STM32F4xx)
        #include "stm32f4xx_hal.h"
    #elif defined(STM32F7xx)
        #include "stm32f7xx_hal.h"
    #elif defined(STM32G0xx)
        #include "stm32g0xx_hal.h"
    #elif defined(STM32G4xx)
        #include "stm32g4xx_hal.h"
    #elif defined(STM32H5xx)
        #include "stm32h5xx_hal.h"
    #elif defined(STM32H7xx)
        #include "stm32h7xx_hal.h"
    #elif defined(STM32L0xx)
        #include "stm32l0xx_hal.h"
    #elif defined(STM32L1xx)
        #include "stm32l1xx_hal.h"
    #elif defined(STM32L4xx)
        #include "stm32l4xx_hal.h"
    #elif defined(STM32L5xx)
        #include "stm32l5xx_hal.h"
    #elif defined(STM32U0xx)
        #include "stm32u0xx_hal.h"
    #elif defined(STM32U5xx)
        #include "stm32u5xx_hal.h"
    #elif defined(STM32WBxx) || defined(STM32WB5Mxx) || defined(STM32WB55xx)
        #include "stm32wbxx_hal.h"
    #elif defined(STM32WBAxx)
        #include "stm32wbaxx_hal.h"
    #elif defined(STM32WLxx)
        #include "stm32wlxx_hal.h"
    #else
        #error "STM32 family not detected. Define DRV8214_STM32_HAL_HEADER in build flags."
    #endif

    /**
     * @brief Set the HAL I2C handle used by all DRV8214 I2C operations.
     * @param hi2c Pointer to an initialized I2C_HandleTypeDef (e.g. &hi2c1).
     */
    void drv8214_i2c_set_handle(I2C_HandleTypeDef* hi2c);
#endif

/**
 * @brief Write a single byte to a device register.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @param value Byte to write.
 */
void drv8214_i2c_write_register(uint8_t device_address, uint8_t reg, uint8_t value);

/**
 * @brief Read a single byte from a device register.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @return The byte read, or 0 on failure.
 */
uint8_t drv8214_i2c_read_register(uint8_t device_address, uint8_t reg);

/**
 * @brief Set or clear specific bits in a register using a mask.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @param mask Bitmask selecting which bits to modify.
 * @param enable_bits Non-zero to set the masked bits, zero to clear them.
 */
void drv8214_i2c_modify_register(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t enable_bits);

/**
 * @brief Replace specific bits in a register with a new value.
 * @param device_address 7-bit I2C address.
 * @param reg Register address.
 * @param mask Bitmask selecting which bits to replace.
 * @param new_value New bit values (only masked bits are applied).
 */
void drv8214_i2c_modify_register_bits(uint8_t device_address, uint8_t reg, uint8_t mask, uint8_t new_value);

#endif /* DRV8214_PLATFORM_I2C_H */
