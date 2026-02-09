/**
 * @file drv8214_platform_config.h
 * @brief Compile-time platform detection for the DRV8214 library.
 *
 * Automatically detects the target platform (Arduino/ESP32 or STM32) via
 * preprocessor macros and includes the appropriate system headers.
 *
 * @copyright Copyright (c) 2025 Theo Heng
 * @license MIT License. See LICENSE file for details.
 */

#ifndef DRV8214_PLATFORM_CONFIG_H
#define DRV8214_PLATFORM_CONFIG_H

#if defined(ESP32) || defined(ESP_PLATFORM) || defined(ARDUINO)
    #define DRV8214_PLATFORM_ARDUINO
    #include <Arduino.h>
#elif defined(STM32WB5Mxx) || defined(STM32F4xx) || defined(USE_STM32_HAL_DRIVER) || defined(USE_HAL_DRIVER)
    #define DRV8214_PLATFORM_STM32
    #include <stdio.h>
    #include <math.h>
#else
    #error "Unsupported platform. Define DRV8214_PLATFORM_ARDUINO or DRV8214_PLATFORM_STM32 manually."
#endif

#endif /* DRV8214_PLATFORM_CONFIG_H */
