/**
 * @file drv8214.h
 * @brief Driver class for the TI DRV8214 brushed DC motor driver.
 *
 * Provides I2C-based control for motor driving, sensorless ripple counting,
 * speed/voltage/current regulation, stall detection, and fault monitoring.
 *
 * @see DRV8214 datasheet: https://www.ti.com/product/DRV8214
 *
 * @copyright Copyright (c) 2025 Theo Heng
 * @license MIT License. See LICENSE file for details.
 */

#ifndef DRV8214_H
#define DRV8214_H

#include "drv8214_platform_config.h"
#include "drv8214_platform_i2c.h"

/** @brief Success return code. */
#define DRV8214_OK          0
/** @brief I2C communication error (NACK, timeout, bus error). */
#define DRV8214_ERR_I2C     1
/** @brief STM32: I2C handle not initialized (call drv8214_i2c_set_handle first). */
#define DRV8214_ERR_HANDLE  2

/* ───────────────────────── I2C Addresses ───────────────────────── */

/** @name I2C Addresses (7-bit, depends on A0/A1 pin settings)
 *  @{ */
#define DRV8214_I2C_ADDR_00  0x30  /**< A1=0,    A0=0     (8-bit: 0x60/0x61) */
#define DRV8214_I2C_ADDR_0Z  0x31  /**< A1=0,    A0=Hi-Z  (8-bit: 0x62/0x63) */
#define DRV8214_I2C_ADDR_01  0x32  /**< A1=0,    A0=1     (8-bit: 0x64/0x65) */
#define DRV8214_I2C_ADDR_Z0  0x33  /**< A1=Hi-Z, A0=0     (8-bit: 0x66/0x67) */
#define DRV8214_I2C_ADDR_ZZ  0x34  /**< A1=Hi-Z, A0=Hi-Z  (8-bit: 0x68/0x69) */
#define DRV8214_I2C_ADDR_Z1  0x35  /**< A1=Hi-Z, A0=1     (8-bit: 0x6A/0x6B) */
#define DRV8214_I2C_ADDR_10  0x36  /**< A1=1,    A0=0     (8-bit: 0x6C/0x6D) */
#define DRV8214_I2C_ADDR_1Z  0x37  /**< A1=1,    A0=Hi-Z  (8-bit: 0x6E/0x6F) */
#define DRV8214_I2C_ADDR_11  0x38  /**< A1=1,    A0=1     (8-bit: 0x70/0x71) */
/** @} */

/* ──────────────────── Status Registers (Read-Only) ──────────────────── */

/** @name Status Registers
 *  @{ */
#define DRV8214_FAULT        0x00  /**< Fault status. */
#define DRV8214_RC_STATUS1   0x01  /**< Ripple-count estimated motor speed. */
#define DRV8214_RC_STATUS2   0x02  /**< Ripple counter [7:0]. */
#define DRV8214_RC_STATUS3   0x03  /**< Ripple counter [15:8]. */
#define DRV8214_REG_STATUS1  0x04  /**< Motor terminal voltage (0x00=0V, 0xB0=11V). */
#define DRV8214_REG_STATUS2  0x05  /**< Motor current (0x00=0A, 0xC0=max per CS_GAIN_SEL). */
#define DRV8214_REG_STATUS3  0x06  /**< Bridge duty cycle (6-bit, 0%..100%). */
/** @} */

/* ──────────────────── Configuration Registers (R/W) ──────────────────── */

/** @name Configuration Registers
 *  @{ */
#define DRV8214_CONFIG0      0x09  /**< General configuration. */
#define DRV8214_CONFIG1      0x0A  /**< Inrush time [15:8]. */
#define DRV8214_CONFIG2      0x0B  /**< Inrush time [7:0]. */
#define DRV8214_CONFIG3      0x0C  /**< Current regulation, stall detection, protection. */
#define DRV8214_CONFIG4      0x0D  /**< Control mode and I2C settings. */
#define DRV8214_REG_CTRL0    0x0E  /**< Soft start, PWM frequency, W_SCALE. */
#define DRV8214_REG_CTRL1    0x0F  /**< Speed/voltage setpoint (WSET_VSET). */
#define DRV8214_REG_CTRL2    0x10  /**< External duty cycle and output filter. */
#define DRV8214_RC_CTRL0     0x11  /**< Ripple counting enable, error correction, CS gain. */
#define DRV8214_RC_CTRL1     0x12  /**< Ripple count threshold [7:0]. */
#define DRV8214_RC_CTRL2     0x13  /**< INV_R scale, KMC scale, threshold scale/high bits. */
#define DRV8214_RC_CTRL3     0x14  /**< Motor inverse resistance (INV_R). */
#define DRV8214_RC_CTRL4     0x15  /**< KMC value. */
#define DRV8214_RC_CTRL5     0x16  /**< Filter coefficient (FLT_K). */
#define DRV8214_RC_CTRL6     0x17  /**< Mechanical fault, error correction settings. */
#define DRV8214_RC_CTRL7     0x18  /**< Proportional gain (KP) for control loop. */
#define DRV8214_RC_CTRL8     0x19  /**< Integral gain (KI) for control loop. */
/** @} */

/* ──────────────────── Bit Masks ──────────────────── */

/** @name FAULT Register (0x00) - Read Only
 *  @{ */
#define FAULT_FAULT          0x80  /**< Bit 7 - General fault (pulls nFAULT low). */
#define FAULT_STALL          0x20  /**< Bit 5 - Stall detected. */
#define FAULT_OCP            0x10  /**< Bit 4 - Overcurrent protection triggered. */
#define FAULT_OVP            0x08  /**< Bit 3 - Overvoltage protection triggered. */
#define FAULT_TSD            0x04  /**< Bit 2 - Thermal shutdown triggered. */
#define FAULT_NPOR           0x02  /**< Bit 1 - Power-on reset occurred. */
#define FAULT_CNT_DONE       0x01  /**< Bit 0 - Ripple count threshold exceeded. */
/** @} */

/** @name REG_STATUS3 (0x06) - Read Only
 *  @{ */
#define REG_STATUS3_IN_DUTY  0x3F  /**< Bits [5:0] - Bridge control duty cycle. */
/** @} */

/** @name CONFIG0 (0x09)
 *  @{ */
#define CONFIG0_EN_OUT       0x80  /**< Bit 7 - Enable driver outputs (0=Hi-Z, 1=enabled). */
#define CONFIG0_EN_OVP       0x40  /**< Bit 6 - Enable overvoltage protection. */
#define CONFIG0_EN_STALL     0x20  /**< Bit 5 - Enable stall detection. */
#define CONFIG0_VSNS_SEL     0x10  /**< Bit 4 - Voltage sensing selection. */
#define CONFIG0_VM_GAIN_SEL  0x08  /**< Bit 3 - Voltage range (0=0..15.7V, 1=0..3.92V). */
#define CONFIG0_CLR_CNT      0x04  /**< Bit 2 - Reset ripple counter (auto-clears). */
#define CONFIG0_CLR_FLT      0x02  /**< Bit 1 - Clear all fault flags (auto-clears). */
#define CONFIG0_DUTY_CTRL    0x01  /**< Bit 0 - Enable duty cycle control mode. */
/** @} */

/** @name CONFIG3 (0x0C)
 *  @{ */
#define CONFIG3_IMODE        0xC0  /**< Bits [7:6] - Current regulation mode. */
#define CONFIG3_SMODE        0x20  /**< Bit 5   - Stall mode (0=disable outputs, 1=keep driving). */
#define CONFIG3_INT_VREF     0x10  /**< Bit 4   - Internal VREF enable (fixed 500mV). */
#define CONFIG3_TBLANK       0x08  /**< Bit 3   - Blanking time (0=1.8us, 1=1.0us). */
#define CONFIG3_TDEG         0x04  /**< Bit 2   - Deglitch time (0=2us, 1=1us). */
#define CONFIG3_OCP_MODE     0x02  /**< Bit 1   - OCP mode (0=latch off, 1=auto-retry). */
#define CONFIG3_TSD_MODE     0x01  /**< Bit 0   - TSD mode. */
/** @} */

/** @name CONFIG4 (0x0D)
 *  @{ */
#define CONFIG4_RC_REP       0xC0  /**< Bits [7:6] - Ripple count reporting mode. */
#define CONFIG4_STALL_REP    0x20  /**< Bit 5   - Stall reporting on nFAULT. */
#define CONFIG4_CBC_REP      0x10  /**< Bit 4   - Cycle-by-cycle current regulation reporting. */
#define CONFIG4_PMODE        0x08  /**< Bit 3   - Control mode (0=PH/EN, 1=PWM). */
#define CONFIG4_I2C_BC       0x04  /**< Bit 2   - Bridge control source (0=INx pins, 1=I2C). */
#define CONFIG4_I2C_EN_IN1   0x02  /**< Bit 1   - I2C bridge control: EN/IN1. */
#define CONFIG4_I2C_PH_IN2   0x01  /**< Bit 0   - I2C bridge control: PH/IN2. */
/** @} */

/** @name REG_CTRL0 (0x0E)
 *  @{ */
#define REG_CTRL0_EN_SS      0x20  /**< Bit 5     - Soft start/stop enable. */
#define REG_CTRL0_REG_CTRL   0x18  /**< Bits [4:3] - Regulation mode. */
#define REG_CTRL0_PWM_FREQ   0x04  /**< Bit 2     - PWM frequency (0=50kHz, 1=25kHz). */
#define REG_CTRL0_W_SCALE    0x03  /**< Bits [1:0] - Speed scaling factor. */
/** @} */

/** @name REG_CTRL2 (0x10)
 *  @{ */
#define REG_CTRL2_OUT_FLT    0xC0  /**< Bits [7:6] - Output filter mode. */
#define REG_CTRL2_EXT_DUTY   0x3F  /**< Bits [5:0] - External duty cycle. */
/** @} */

/** @name RC_CTRL0 (0x11)
 *  @{ */
#define RC_CTRL0_EN_RC        0x80  /**< Bit 7     - Enable ripple counting. */
#define RC_CTRL0_DIS_EC       0x40  /**< Bit 6     - Disable error correction. */
#define RC_CTRL0_RC_HIZ       0x20  /**< Bit 5     - H-bridge Hi-Z when count exceeds threshold. */
#define RC_CTRL0_FLT_GAIN_SEL 0x18  /**< Bits [4:3] - Filter gain selection. */
#define RC_CTRL0_CS_GAIN_SEL  0x07  /**< Bits [2:0] - Current sense gain selection. */
/** @} */

/** @name RC_CTRL2 (0x13)
 *  @{ */
#define RC_CTRL2_INV_R_SCALE  0xC0  /**< Bits [7:6] - Inverse resistance scale. */
#define RC_CTRL2_KMC_SCALE    0x30  /**< Bits [5:4] - KMC scaling factor. */
#define RC_CTRL2_RC_THR_SCALE 0x0C  /**< Bits [3:2] - Ripple threshold scaling. */
#define RC_CTRL2_RC_THR_HIGH  0x03  /**< Bits [1:0] - Ripple threshold [9:8]. */
/** @} */

/** @name RC_CTRL5 (0x16)
 *  @{ */
#define RC_CTRL5_FLT_K        0xF0  /**< Bits [7:4] - Filter coefficient. */
/** @} */

/** @name RC_CTRL6 (0x17)
 *  @{ */
#define RC_CTRL6_EC_PULSE_DIS 0x80  /**< Bit 7     - Disable error correction pulse. */
#define RC_CTRL6_T_MECH_FLT   0x70  /**< Bits [6:4] - Mechanical fault detection time. */
#define RC_CTRL6_EC_FALSE_PER 0x0C  /**< Bits [3:2] - Error correction false period. */
#define RC_CTRL6_EC_MISS_PER  0x03  /**< Bits [1:0] - Error correction miss period. */
/** @} */

/** @name RC_CTRL7 (0x18)
 *  @{ */
#define RC_CTRL7_KP_DIV       0xE0  /**< Bits [7:5] - Proportional gain divisor. */
#define RC_CTRL7_KP           0x1F  /**< Bits [4:0] - Proportional gain value. */
/** @} */

/** @name RC_CTRL8 (0x19)
 *  @{ */
#define RC_CTRL8_KI_DIV       0xE0  /**< Bits [7:5] - Integral gain divisor. */
#define RC_CTRL8_KI           0x1F  /**< Bits [4:0] - Integral gain value. */
/** @} */

/* ──────────────────── Enumerations ──────────────────── */

/** @brief H-bridge control mode. */
enum ControlMode {
    PWM,   /**< PWM mode (IN1/IN2 determine direction + duty). */
    PH_EN  /**< Phase/Enable mode (PH=direction, EN=drive/brake). */
};

/**
 * @brief Regulation mode for the internal control loop.
 * @note In CURRENT_FIXED and CURRENT_CYCLES modes, speed/voltage cannot be
 *       controlled via I2C (full supply voltage is applied to the motor).
 */
enum RegulationMode {
    CURRENT_FIXED,   /**< Fixed off-time current regulation. */
    CURRENT_CYCLES,  /**< Cycle-by-cycle current regulation. */
    SPEED,           /**< Closed-loop speed regulation via ripple counting. */
    VOLTAGE          /**< Closed-loop voltage regulation. */
};

/* ──────────────────── Configuration ──────────────────── */

/**
 * @brief Configuration parameters for the DRV8214 driver.
 *
 * All fields have sensible defaults. Pass to DRV8214::init() to apply.
 */
struct DRV8214_Config {
    bool I2CControlled = true;               /**< Bridge controlled via I2C (vs. INx pins). */
    ControlMode control_mode = PWM;          /**< H-bridge control mode. */
    RegulationMode regulation_mode = SPEED;  /**< Regulation loop mode. */
    bool voltage_range = true;               /**< Voltage range (false=0..15.7V, true=0..3.92V). */
    float Vref = 0.5f;                       /**< Current-sense reference voltage [V]. */
    bool stall_enabled = true;               /**< Enable stall detection. */
    bool ovp_enabled = true;                 /**< Enable overvoltage protection. */
    bool stall_behavior = false;             /**< Stall action (false=disable outputs, true=keep driving). */
    bool bridge_behavior_thr_reached = false; /**< At ripple threshold (false=keep driving, true=Hi-Z). */
    uint8_t current_reg_mode = 3;            /**< IMODE [1:0]: 0=none, 1=inrush only, 2-3=always. */
    float Aipropri = 0;                      /**< Current mirror gain [A/A] (set by CS_GAIN_SEL). */
    float Itrip = 0.0f;                      /**< Calculated trip current [A]. */
    float MaxCurrent = 0.0f;                 /**< Maximum current [A] (set by CS_GAIN_SEL). */
    uint8_t w_scale = 128;                   /**< Speed scaling factor for WSET_VSET. */
    bool verbose = false;                    /**< Print debug information via drvPrint(). */
    uint16_t inrush_duration = 500;          /**< Stall-detection blanking time [ms]. */
    uint8_t inv_r = 0;                       /**< Inverse motor resistance register value. */
    uint16_t inv_r_scale = 0;               /**< Inverse resistance scale factor. */
    uint8_t kmc = 30;                        /**< KMC register value. */
    uint8_t kmc_scale = 0b11;               /**< KMC scale bits. */
    bool soft_start_stop_enabled = false;    /**< Enable soft start/stop ramp. */
    uint16_t ripple_threshold = 0;           /**< Ripple count threshold (before scaling). */
    uint8_t ripple_threshold_scale = 2;      /**< Ripple threshold scale bits. */
};

/* ──────────────────── Driver Class ──────────────────── */

/**
 * @brief Driver class for the TI DRV8214 brushed DC motor driver.
 *
 * Encapsulates I2C register access, motor control, ripple counting,
 * regulation mode configuration, and fault monitoring.
 */
class DRV8214 {

private:
    uint8_t  address;                   /**< 7-bit I2C address. */
    uint8_t  driver_ID;                 /**< Identifier when using multiple drivers. */
    uint16_t Ripropri;                  /**< IPROPI sense resistor [Ohms]. */
    uint16_t ripples_per_revolution;    /**< Current ripples per rotor revolution. */
    uint8_t  motor_internal_resistance; /**< Motor winding resistance [Ohms]. */
    uint8_t  motor_reduction_ratio;     /**< Gearbox reduction ratio. */
    uint16_t motor_max_rpm;             /**< Maximum motor RPM. */

    DRV8214_Config config;

#ifdef DRV8214_PLATFORM_ARDUINO
    Stream* _debugPort = nullptr;
#endif

    void drvPrint(const char* message);

public:
    /**
     * @brief Construct a DRV8214 driver instance.
     * @param addr   7-bit I2C address (use DRV8214_I2C_ADDR_xx defines).
     * @param id     Driver identifier (for multi-driver setups).
     * @param sense_resistor  IPROPI resistor value [Ohms].
     * @param ripples         Ripples per rotor revolution.
     * @param rm              Motor winding resistance [Ohms].
     * @param reduction_ratio Gearbox reduction ratio.
     * @param rpm             Maximum motor RPM.
     */
    DRV8214(uint8_t addr, uint8_t id, uint16_t sense_resistor, uint8_t ripples,
            uint8_t rm, uint8_t reduction_ratio, uint16_t rpm)
        : address(addr), driver_ID(id), Ripropri(sense_resistor),
          ripples_per_revolution(ripples), motor_internal_resistance(rm),
          motor_reduction_ratio(reduction_ratio), motor_max_rpm(rpm) {}

    /**
     * @brief Initialize the driver with the given configuration.
     * @param config Configuration parameters.
     * @return DRV8214_OK on success.
     */
    uint8_t init(const DRV8214_Config& config);

    /** @name Status Getters
     *  @{ */
    uint8_t  getDriverAddress();
    uint8_t  getDriverID();
    uint8_t  getSenseResistor();
    uint8_t  getRipplesPerRevolution();
    uint8_t  getFaultStatus();          /**< Read the FAULT register. */
    uint32_t getMotorSpeedRPM();        /**< Rotor speed [RPM] from ripple count. */
    uint16_t getMotorSpeedRAD();        /**< Rotor speed [rad/s] from ripple count. */
    uint16_t getMotorSpeedShaftRPM();   /**< Output shaft speed [RPM] (after reduction). */
    uint16_t getMotorSpeedShaftRAD();   /**< Output shaft speed [rad/s] (after reduction). */
    uint8_t  getMotorSpeedRegister();   /**< Raw RC_STATUS1 register value. */
    uint16_t getRippleCount();          /**< 16-bit ripple counter. */
    float    getMotorVoltage();         /**< Motor voltage [V] (scaled by voltage range). */
    uint8_t  getMotorVoltageRegister(); /**< Raw REG_STATUS1 register value. */
    float    getMotorCurrent();         /**< Motor current [A] (scaled by CS_GAIN_SEL). */
    uint8_t  getMotorCurrentRegister(); /**< Raw REG_STATUS2 register value. */
    uint8_t  getDutyCycle();            /**< Bridge duty cycle [0..100 %]. */
    /** @} */

    /** @name Raw Register Getters
     *  @{ */
    uint8_t  getCONFIG0();
    uint16_t getInrushDuration();
    uint8_t  getCONFIG3();
    uint8_t  getCONFIG4();
    uint8_t  getREG_CTRL0();
    uint8_t  getREG_CTRL1();
    uint8_t  getREG_CTRL2();
    uint8_t  getRC_CTRL0();
    uint8_t  getRC_CTRL1();
    uint8_t  getRC_CTRL2();
    uint16_t getRippleThreshold();       /**< 10-bit ripple threshold (unscaled). */
    uint16_t getRippleThresholdScaled(); /**< Effective threshold (threshold x scale). */
    uint16_t getRippleThresholdScale();  /**< Threshold scale factor bits. */
    uint8_t  getKMC();
    uint8_t  getKMCScale();
    uint8_t  getFilterDamping();
    uint8_t  getRC_CTRL6();
    uint8_t  getRC_CTRL7();
    uint8_t  getRC_CTRL8();
    /** @} */

    /** @name Configuration Functions
     *  @{ */
    void enableHbridge();
    void disableHbridge();
    void setStallDetection(bool stall_en);
    void setVoltageRange(bool range);
    void setOvervoltageProtection(bool ovp);
    void resetRippleCounter();
    void resetFaultFlags();
    void enableDutyCycleControl();
    void disableDutyCycleControl();
    void setInrushDuration(uint16_t inrush_dur);
    void setCurrentRegMode(uint8_t mode);
    void setStallBehavior(bool behavior);
    void setInternalVoltageReference(float reference_voltage);
    void configureConfig3(uint8_t config3);
    void setI2CControl(bool I2CControl);
    void enablePWMControl();
    void enablePHENControl();
    void enableStallInterrupt();
    void disableStallInterrupt();
    void enableCountThresholdInterrupt();
    void disableCountThresholdInterrupt();
    void setBridgeBehaviorThresholdReached(bool stops);
    void setSoftStartStop(bool enable);
    void configureControl0(uint8_t control0);

    /**
     * @brief Set the target speed for SPEED regulation mode.
     * @param speed Target speed in RPM (capped at motor_max_rpm).
     */
    void setRippleSpeed(uint16_t speed);

    /**
     * @brief Set the target voltage for VOLTAGE regulation mode.
     * @param voltage Target voltage [V] (clamped to the configured range).
     */
    void setVoltageSpeed(float voltage);

    /**
     * @brief Configure CS_GAIN_SEL to match the requested current.
     *
     * Selects the appropriate current sense gain and updates Aipropri,
     * MaxCurrent, and Itrip in the stored configuration.
     *
     * @param requested_current Desired regulation/stall current [A].
     */
    void setRegulationAndStallCurrent(float requested_current);

    void configureControl2(uint8_t control2);
    void enableRippleCount(bool enable = true);
    void enableErrorCorrection(bool enable = true);
    void configureRippleCount0(uint8_t ripple0);
    void setRippleCountThreshold(uint16_t threshold);
    void setRippleThresholdScale(uint8_t scale);
    void setKMCScale(uint8_t scale);
    void setMotorInverseResistance(uint8_t resistance);
    void setMotorInverseResistanceScale(uint8_t scale);
    void setResistanceRelatedParameters();
    void setKMC(uint8_t factor);
    void setFilterDamping(uint8_t damping);
    void configureRippleCount6(uint8_t ripple6);
    void configureRippleCount7(uint8_t ripple7);
    void configureRippleCount8(uint8_t ripple8);
    /** @} */

    /** @name Motor Control
     *  @{ */

    /**
     * @brief Select the H-bridge control mode and I2C bridge control.
     * @param mode       PWM or PH_EN.
     * @param I2CControl true = bridge controlled via I2C bits.
     */
    void setControlMode(ControlMode mode, bool I2CControl);

    /** @brief Set the regulation loop mode (current / speed / voltage). */
    void setRegulationMode(RegulationMode regulation);

    /**
     * @brief Drive the motor forward.
     * @param speed             Target RPM (SPEED mode).
     * @param voltage           Target voltage [V] (VOLTAGE mode).
     * @param requested_current Target current [A] (CURRENT modes).
     */
    void turnForward(uint16_t speed = 0, float voltage = 0, float requested_current = 0);

    /**
     * @brief Drive the motor in reverse.
     * @param speed             Target RPM (SPEED mode).
     * @param voltage           Target voltage [V] (VOLTAGE mode).
     * @param requested_current Target current [A] (CURRENT modes).
     */
    void turnReverse(uint16_t speed = 0, float voltage = 0, float requested_current = 0);

    /**
     * @brief Active-brake the motor (low-side FETs on).
     * @param initial_config If true, suppress verbose output.
     */
    void brakeMotor(bool initial_config = false);

    /** @brief Coast the motor (Hi-Z, PWM mode only). */
    void coastMotor();

    /**
     * @brief Rotate the motor for a given number of ripples.
     * @param ripples_target Number of current ripples to count.
     * @param stops          true = H-bridge disables at threshold.
     * @param direction      true = forward, false = reverse.
     * @param speed          Target RPM (SPEED mode).
     * @param voltage        Target voltage [V] (VOLTAGE mode).
     * @param current        Target current [A] (CURRENT modes).
     */
    void turnXRipples(uint16_t ripples_target, bool stops = true,
                      bool direction = true, uint16_t speed = 0,
                      float voltage = 0, float current = 0);

    /**
     * @brief Rotate the motor for a given number of output shaft revolutions.
     * @param revolutions_target Number of output shaft revolutions.
     * @param stops              true = H-bridge disables at threshold.
     * @param direction          true = forward, false = reverse.
     * @param speed              Target RPM (SPEED mode).
     * @param voltage            Target voltage [V] (VOLTAGE mode).
     * @param current            Target current [A] (CURRENT modes).
     */
    void turnXRevolutions(uint16_t revolutions_target, bool stops = true,
                          bool direction = true, uint16_t speed = 0,
                          float voltage = 0, float current = 0);
    /** @} */

    /** @name Debug / Diagnostics
     *  @{ */
    void printMotorConfig(bool initial_config = false);
    void printFaultStatus();
#ifdef DRV8214_PLATFORM_ARDUINO
    /** @brief Set the output stream for debug messages (e.g. &Serial). */
    void setDebugStream(Stream* debugPort);
#endif
    /** @} */
};

#endif /* DRV8214_H */
