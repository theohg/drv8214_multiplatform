/**
 * @file drv8214.cpp
 * @brief Implementation of the DRV8214 driver class.
 *
 * @copyright Copyright (c) 2026 Theo Heng
 * @license MIT License. See LICENSE file for details.
 */

#include "drv8214.h"

/* ──────────────────── Initialization ──────────────────── */

uint8_t DRV8214::init(const DRV8214_Config& cfg) {
    config = cfg;

    disableHbridge();
    setControlMode(config.control_mode, config.I2CControlled);
    setRegulationMode(config.regulation_mode);
    setVoltageRange(config.voltage_range);
    setOvervoltageProtection(config.ovp_enabled);
    setCurrentRegMode(config.current_reg_mode);
    setStallDetection(config.stall_enabled);
    setStallBehavior(config.stall_behavior);
    enableStallInterrupt();
    enableCountThresholdInterrupt();
    setBridgeBehaviorThresholdReached(config.bridge_behavior_thr_reached);
    setInternalVoltageReference(0);
    setSoftStartStop(config.soft_start_stop_enabled);
    setInrushDuration(config.inrush_duration);
    setResistanceRelatedParameters();
    enableRippleCount();
    resetRippleCounter();
    setKMC(config.kmc);
    setKMCScale(config.kmc_scale);
    brakeMotor(true);
    enableErrorCorrection(false);

    if (config.verbose) { printMotorConfig(true); }

    return DRV8214_OK;
}

/* ──────────────────── Status Getters ──────────────────── */

uint8_t DRV8214::getDriverAddress() {
    return address;
}

uint8_t DRV8214::getDriverID() {
    return driver_ID;
}

uint8_t DRV8214::getSenseResistor() {
    return Ripropri;
}

uint8_t DRV8214::getRipplesPerRevolution() {
    return ripples_per_revolution;
}

uint8_t DRV8214::getFaultStatus() {
    return drv8214_i2c_read_register(address, DRV8214_FAULT);
}

uint32_t DRV8214::getMotorSpeedRPM() {
    return ((drv8214_i2c_read_register(address, DRV8214_RC_STATUS1) * config.w_scale * 60)
            / (2 * M_PI * ripples_per_revolution));
}

uint16_t DRV8214::getMotorSpeedRAD() {
    return ((drv8214_i2c_read_register(address, DRV8214_RC_STATUS1) * config.w_scale)
            / ripples_per_revolution);
}

uint16_t DRV8214::getMotorSpeedShaftRPM() {
    return (getMotorSpeedRPM() / motor_reduction_ratio);
}

uint16_t DRV8214::getMotorSpeedShaftRAD() {
    return (getMotorSpeedRAD() / motor_reduction_ratio);
}

uint8_t DRV8214::getMotorSpeedRegister() {
    return drv8214_i2c_read_register(address, DRV8214_RC_STATUS1);
}

uint16_t DRV8214::getRippleCount() {
    return (drv8214_i2c_read_register(address, DRV8214_RC_STATUS3) << 8)
         | drv8214_i2c_read_register(address, DRV8214_RC_STATUS2);
}

float DRV8214::getMotorVoltage() {
    if (config.voltage_range) {
        return (drv8214_i2c_read_register(address, DRV8214_REG_STATUS1) / 255.0f) * 3.92f;
    } else {
        if (config.ovp_enabled) {
            uint8_t raw = drv8214_i2c_read_register(address, DRV8214_REG_STATUS1);
            if (raw > 0xB0) { return 11.0f; }
            return (raw / 176.0f) * 11.0f;
        } else {
            return (drv8214_i2c_read_register(address, DRV8214_REG_STATUS1) / 255.0f) * 15.7f;
        }
    }
}

uint8_t DRV8214::getMotorVoltageRegister() {
    return drv8214_i2c_read_register(address, DRV8214_REG_STATUS1);
}

float DRV8214::getMotorCurrent() {
    return (drv8214_i2c_read_register(address, DRV8214_REG_STATUS2) / 192.0f) * config.MaxCurrent;
}

uint8_t DRV8214::getMotorCurrentRegister() {
    return drv8214_i2c_read_register(address, DRV8214_REG_STATUS2);
}

uint8_t DRV8214::getDutyCycle() {
    uint8_t dutyCycle = drv8214_i2c_read_register(address, DRV8214_REG_STATUS3) & REG_STATUS3_IN_DUTY;
    return (dutyCycle * 100) / 63;
}

/* ──────────────────── Raw Register Getters ──────────────────── */

uint8_t DRV8214::getCONFIG0() {
    return drv8214_i2c_read_register(address, DRV8214_CONFIG0);
}

uint16_t DRV8214::getInrushDuration() {
    return (drv8214_i2c_read_register(address, DRV8214_CONFIG1) << 8)
         | drv8214_i2c_read_register(address, DRV8214_CONFIG2);
}

uint8_t DRV8214::getCONFIG3() {
    return drv8214_i2c_read_register(address, DRV8214_CONFIG3);
}

uint8_t DRV8214::getCONFIG4() {
    return drv8214_i2c_read_register(address, DRV8214_CONFIG4);
}

uint8_t DRV8214::getREG_CTRL0() {
    return drv8214_i2c_read_register(address, DRV8214_REG_CTRL0);
}

uint8_t DRV8214::getREG_CTRL1() {
    return drv8214_i2c_read_register(address, DRV8214_REG_CTRL1);
}

uint8_t DRV8214::getREG_CTRL2() {
    return drv8214_i2c_read_register(address, DRV8214_REG_CTRL2);
}

uint8_t DRV8214::getRC_CTRL0() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL0);
}

uint8_t DRV8214::getRC_CTRL1() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL1);
}

uint8_t DRV8214::getRC_CTRL2() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL2);
}

uint16_t DRV8214::getRippleThreshold() {
    uint8_t ctrl2 = drv8214_i2c_read_register(address, DRV8214_RC_CTRL2);
    uint8_t ctrl1 = drv8214_i2c_read_register(address, DRV8214_RC_CTRL1);
    uint16_t thr_high = (ctrl2 & 0x03) << 8;
    uint16_t thr_low  = ctrl1;
    return (thr_high | thr_low);
}

uint16_t DRV8214::getRippleThresholdScaled() {
    getRippleThresholdScale();
    if (config.ripple_threshold_scale == 0) { return getRippleThreshold() * 2;  }
    if (config.ripple_threshold_scale == 1) { return getRippleThreshold() * 8;  }
    if (config.ripple_threshold_scale == 2) { return getRippleThreshold() * 16; }
    if (config.ripple_threshold_scale == 3) { return getRippleThreshold() * 64; }
    return getRippleThreshold() * config.ripple_threshold_scale;
}

uint16_t DRV8214::getRippleThresholdScale() {
    config.ripple_threshold_scale = (drv8214_i2c_read_register(address, DRV8214_RC_CTRL2) & RC_CTRL2_RC_THR_SCALE) >> 2;
    return config.ripple_threshold_scale;
}

uint8_t DRV8214::getKMC() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL4);
}

uint8_t DRV8214::getKMCScale() {
    return (drv8214_i2c_read_register(address, DRV8214_RC_CTRL2) >> 4) & 0x03;
}

uint8_t DRV8214::getFilterDamping() {
    return (drv8214_i2c_read_register(address, DRV8214_RC_CTRL5) >> 4) & 0x0F;
}

uint8_t DRV8214::getRC_CTRL6() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL6);
}

uint8_t DRV8214::getRC_CTRL7() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL7);
}

uint8_t DRV8214::getRC_CTRL8() {
    return drv8214_i2c_read_register(address, DRV8214_RC_CTRL8);
}

/* ──────────────────── Configuration Functions ──────────────────── */

void DRV8214::enableHbridge() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_EN_OUT, true);
}

void DRV8214::disableHbridge() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_EN_OUT, false);
}

void DRV8214::setStallDetection(bool stall_en) {
    config.stall_enabled = stall_en;
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_EN_STALL, stall_en);
}

void DRV8214::setVoltageRange(bool range) {
    config.voltage_range = range;
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_VM_GAIN_SEL, range);
}

void DRV8214::setOvervoltageProtection(bool ovp) {
    config.ovp_enabled = ovp;
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_EN_OVP, ovp);
}

void DRV8214::resetRippleCounter() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_CLR_CNT, true);
}

void DRV8214::resetFaultFlags() {
    disableHbridge();
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_CLR_FLT, true);
    enableHbridge();
}

void DRV8214::enableDutyCycleControl() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_DUTY_CTRL, true);
}

void DRV8214::disableDutyCycleControl() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG0, CONFIG0_DUTY_CTRL, false);
}

void DRV8214::setInrushDuration(uint16_t threshold) {
    drv8214_i2c_write_register(address, DRV8214_CONFIG1, (threshold >> 8) & 0xFF);
    drv8214_i2c_write_register(address, DRV8214_CONFIG2, threshold & 0xFF);
}

void DRV8214::setCurrentRegMode(uint8_t mode) {
    if (mode > 3) { mode = 3; }
    config.current_reg_mode = mode;
    uint8_t reg_value = static_cast<uint8_t>(mode) << 6;
    drv8214_i2c_modify_register_bits(address, DRV8214_CONFIG3, CONFIG3_IMODE, reg_value);
}

void DRV8214::setStallBehavior(bool behavior) {
    config.stall_behavior = behavior;
    drv8214_i2c_modify_register(address, DRV8214_CONFIG3, CONFIG3_SMODE, behavior);
}

void DRV8214::setInternalVoltageReference(float reference_voltage) {
    if (reference_voltage == 0) {
        config.Vref = 0.5f;
        drv8214_i2c_modify_register(address, DRV8214_CONFIG3, CONFIG3_INT_VREF, true);
    } else {
        config.Vref = reference_voltage;
        drv8214_i2c_modify_register(address, DRV8214_CONFIG3, CONFIG3_INT_VREF, false);
    }
}

void DRV8214::configureConfig3(uint8_t config3) {
    drv8214_i2c_write_register(address, DRV8214_CONFIG3, config3);
}

void DRV8214::setI2CControl(bool I2CControl) {
    config.I2CControlled = I2CControl;
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_BC, I2CControl);
}

void DRV8214::enablePWMControl() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_PMODE, true);
}

void DRV8214::enablePHENControl() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_PMODE, false);
}

void DRV8214::enableStallInterrupt() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_STALL_REP, true);
}

void DRV8214::disableStallInterrupt() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_STALL_REP, false);
}

void DRV8214::enableCountThresholdInterrupt() {
    drv8214_i2c_modify_register_bits(address, DRV8214_CONFIG4, CONFIG4_RC_REP, 0b10000000);
}

void DRV8214::disableCountThresholdInterrupt() {
    drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_RC_REP, false);
}

void DRV8214::setBridgeBehaviorThresholdReached(bool stops) {
    config.bridge_behavior_thr_reached = stops;
    drv8214_i2c_modify_register(address, DRV8214_RC_CTRL0, RC_CTRL0_RC_HIZ, stops);
}

void DRV8214::setSoftStartStop(bool enable) {
    drv8214_i2c_modify_register(address, DRV8214_REG_CTRL0, REG_CTRL0_EN_SS, enable);
}

void DRV8214::configureControl0(uint8_t control0) {
    drv8214_i2c_write_register(address, DRV8214_REG_CTRL0, control0);
}

void DRV8214::setRegulationAndStallCurrent(float requested_current) {
    /* CS_GAIN_SEL settings (Table 8-7):
     *   000 =>  225 uA/A, max 4.0  A
     *   001 =>  225 uA/A, max 2.0  A
     *   010 => 1125 uA/A, max 1.0  A
     *   011 => 1125 uA/A, max 0.5  A
     *   1X0 => 5560 uA/A, max 0.25 A
     *   1X1 => 5560 uA/A, max 0.125A
     */
    uint8_t cs_gain_sel;

    if (requested_current < 0.125f) {
        cs_gain_sel = 0b111;
        config.Aipropri = 5560e-6;
        config.MaxCurrent = 0.125f;
    } else if (requested_current < 0.25f) {
        cs_gain_sel = 0b110;
        config.Aipropri = 5560e-6;
        config.MaxCurrent = 0.25f;
    } else if (requested_current < 0.5f) {
        cs_gain_sel = 0b011;
        config.Aipropri = 1125e-6;
        config.MaxCurrent = 0.5f;
    } else if (requested_current < 1.0f) {
        cs_gain_sel = 0b010;
        config.Aipropri = 1125e-6;
        config.MaxCurrent = 1.0f;
    } else if (requested_current < 2.0f) {
        cs_gain_sel = 0b001;
        config.Aipropri = 225e-6;
        config.MaxCurrent = 2.0f;
    } else {
        cs_gain_sel = 0b000;
        config.Aipropri = 225e-6;
        config.MaxCurrent = 4.0f;
    }

    drv8214_i2c_modify_register_bits(address, DRV8214_RC_CTRL0, RC_CTRL0_CS_GAIN_SEL, cs_gain_sel);
    config.Itrip = config.Vref / (Ripropri * config.Aipropri);

    if (config.verbose) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "CS_GAIN_SEL: 0b%03d | Aipropri: %.0f uA/A | Itrip: %.3f A\n",
                 cs_gain_sel, config.Aipropri * 1e6, config.Itrip);
        drvPrint(buffer);
    }
}

void DRV8214::setRippleSpeed(uint16_t speed) {
    if (speed > motor_max_rpm) { speed = motor_max_rpm; }

    uint32_t ripple_speed = (speed * motor_reduction_ratio * ripples_per_revolution * 2 * M_PI) / 60;

    const uint16_t MAX_SPEED = 32640; // 255 * 128
    if (ripple_speed > MAX_SPEED) { ripple_speed = MAX_SPEED; }

    struct ScaleOption { uint16_t scale; uint8_t bits; };
    ScaleOption scaleOptions[] = {
        {16, 0b00}, {32, 0b01}, {64, 0b10}, {128, 0b11}
    };

    uint16_t WSET_VSET = ripple_speed;
    uint8_t W_SCALE = 0b00;

    for (const auto& option : scaleOptions) {
        if (ripple_speed >= option.scale) {
            WSET_VSET = ripple_speed / option.scale;
            if (WSET_VSET <= 255) {
                W_SCALE = option.bits;
                break;
            }
        }
    }
    config.w_scale = scaleOptions[W_SCALE].scale;
    WSET_VSET = WSET_VSET & 0xFF;

    if (config.verbose) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer),
                 "WSET_VSET: %d | W_SCALE: %d | Target: %d rad/s\n",
                 WSET_VSET, config.w_scale, WSET_VSET * config.w_scale);
        drvPrint(buffer);
    }

    drv8214_i2c_write_register(address, DRV8214_REG_CTRL1, WSET_VSET);
    drv8214_i2c_modify_register_bits(address, DRV8214_REG_CTRL0, REG_CTRL0_W_SCALE, W_SCALE);
}

void DRV8214::setVoltageSpeed(float voltage) {
    if (voltage < 0.0f) { voltage = 0.0f; }

    if (config.voltage_range) {
        // VM_GAIN_SEL=1: 0..3.92V
        if (voltage > 3.92f) { voltage = 3.92f; }
        float scaled = voltage * (255.0f / 3.92f);
        drv8214_i2c_write_register(address, DRV8214_REG_CTRL1, static_cast<uint8_t>(scaled + 0.5f));
    } else {
        // VM_GAIN_SEL=0: 0..15.7V (OVP caps at 11V)
        if (voltage > 15.7f) { voltage = 11.0f; }
        float scaled = voltage * (255.0f / 15.7f);
        drv8214_i2c_write_register(address, DRV8214_REG_CTRL1, static_cast<uint8_t>(scaled + 0.5f));
    }
}

void DRV8214::configureControl2(uint8_t control2) {
    drv8214_i2c_write_register(address, DRV8214_REG_CTRL2, control2);
}

void DRV8214::enableRippleCount(bool enable) {
    drv8214_i2c_modify_register(address, DRV8214_RC_CTRL0, RC_CTRL0_EN_RC, enable);
}

void DRV8214::enableErrorCorrection(bool enable) {
    drv8214_i2c_modify_register(address, DRV8214_RC_CTRL0, RC_CTRL0_DIS_EC, !enable);
}

void DRV8214::configureRippleCount0(uint8_t ripple0) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL0, ripple0);
}

void DRV8214::setRippleCountThreshold(uint16_t threshold) {
    const uint16_t MAX_THRESHOLD = 65535;
    if (threshold > MAX_THRESHOLD) { threshold = MAX_THRESHOLD; }

    struct ScaleOption { uint16_t scale; uint8_t bits; };
    ScaleOption scaleOptions[] = {
        {2, 0b00}, {8, 0b01}, {16, 0b10}, {64, 0b11}
    };

    uint16_t rc_thr = threshold;
    uint8_t rc_thr_scale_bits = 0b00;

    for (const auto& option : scaleOptions) {
        if (threshold >= option.scale) {
            rc_thr = threshold / option.scale;
            if (rc_thr < 1024) {
                rc_thr_scale_bits = option.bits;
                break;
            }
        }
    }

    if (config.verbose) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "RC_THR: %d | RC_THR_SCALE: %d\n", rc_thr, rc_thr_scale_bits);
        drvPrint(buffer);
    }

    config.ripple_threshold = rc_thr;
    config.ripple_threshold_scale = rc_thr_scale_bits;

    rc_thr = rc_thr & 0x3FF;
    uint8_t rc_thr_low  = rc_thr & 0xFF;
    uint8_t rc_thr_high = (rc_thr >> 8) & 0x03;

    drv8214_i2c_write_register(address, DRV8214_RC_CTRL1, rc_thr_low);
    setRippleThresholdScale(rc_thr_scale_bits);
    drv8214_i2c_modify_register_bits(address, DRV8214_RC_CTRL2, RC_CTRL2_RC_THR_HIGH, rc_thr_high);
}

void DRV8214::setRippleThresholdScale(uint8_t scale) {
    scale = (scale & 0x03) << 2;
    drv8214_i2c_modify_register_bits(address, DRV8214_RC_CTRL2, RC_CTRL2_RC_THR_SCALE, scale);
}

void DRV8214::setKMCScale(uint8_t scale) {
    scale = scale << 4;
    drv8214_i2c_modify_register_bits(address, DRV8214_RC_CTRL2, RC_CTRL2_KMC_SCALE, scale);
}

void DRV8214::setMotorInverseResistance(uint8_t resistance) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL3, resistance);
}

void DRV8214::setMotorInverseResistanceScale(uint8_t scale) {
    scale = scale << 6;
    drv8214_i2c_modify_register_bits(address, DRV8214_RC_CTRL2, RC_CTRL2_INV_R_SCALE, scale);
}

void DRV8214::setResistanceRelatedParameters() {
    const uint16_t scaleValues[4] = {2, 64, 1024, 8192};
    const uint8_t scaleBits[4] = {0b00, 0b01, 0b10, 0b11};

    uint8_t bestScaleBits = 0b00;
    uint8_t bestInvR = 1;

    for (int i = 3; i >= 0; --i) {
        float candidate = scaleValues[i] / motor_internal_resistance;
        float rounded = roundf(candidate);
        if (rounded < 1.0f) { rounded = 1.0f; }
        if (rounded <= 255.0f) {
            bestScaleBits = scaleBits[i];
            bestInvR = static_cast<uint8_t>(rounded);
            break;
        }
    }

    config.inv_r = bestInvR;
    config.inv_r_scale = scaleValues[bestScaleBits];

    setMotorInverseResistanceScale(bestScaleBits);
    setMotorInverseResistance(bestInvR);
}

void DRV8214::setKMC(uint8_t factor) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL4, factor);
}

void DRV8214::setFilterDamping(uint8_t damping) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL5, damping);
}

void DRV8214::configureRippleCount6(uint8_t ripple6) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL6, ripple6);
}

void DRV8214::configureRippleCount7(uint8_t ripple7) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL7, ripple7);
}

void DRV8214::configureRippleCount8(uint8_t ripple8) {
    drv8214_i2c_write_register(address, DRV8214_RC_CTRL8, ripple8);
}

/* ──────────────────── Motor Control ──────────────────── */

void DRV8214::setControlMode(ControlMode mode, bool I2CControl) {
    config.control_mode = mode;
    setI2CControl(I2CControl);
    switch (mode) {
        case PWM:   enablePWMControl();  break;
        case PH_EN: enablePHENControl(); break;
    }
}

void DRV8214::setRegulationMode(RegulationMode regulation) {
    uint8_t reg_ctrl = 0;
    switch (regulation) {
        case CURRENT_FIXED:  reg_ctrl = (0b00 << 3); break;
        case CURRENT_CYCLES: reg_ctrl = (0b01 << 3); break;
        case SPEED:          reg_ctrl = (0b10 << 3); enableRippleCount(); break;
        case VOLTAGE:        reg_ctrl = (0b11 << 3); break;
    }
    config.regulation_mode = regulation;
    drv8214_i2c_modify_register_bits(address, DRV8214_REG_CTRL0, REG_CTRL0_REG_CTRL, reg_ctrl);
}

void DRV8214::turnForward(uint16_t speed, float voltage, float requested_current) {
    disableHbridge();

    switch (config.regulation_mode) {
        case CURRENT_FIXED:
        case CURRENT_CYCLES:
            setRegulationAndStallCurrent(requested_current);
            break;
        case SPEED:
            setRippleSpeed(speed);
            break;
        case VOLTAGE:
            setVoltageSpeed(voltage);
            break;
    }

    if (config.control_mode == PWM) {
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, true);
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, false);
    } else {
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, true);
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, true);
    }

    enableHbridge();
    if (config.verbose) { drvPrint("Turning Forward\n"); }
}

void DRV8214::turnReverse(uint16_t speed, float voltage, float requested_current) {
    disableHbridge();

    switch (config.regulation_mode) {
        case CURRENT_FIXED:
        case CURRENT_CYCLES:
            setRegulationAndStallCurrent(requested_current);
            break;
        case SPEED:
            setRippleSpeed(speed);
            break;
        case VOLTAGE:
            setVoltageSpeed(voltage);
            break;
    }

    if (config.control_mode == PWM) {
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, false);
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, true);
    } else {
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, true);
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, false);
    }

    enableHbridge();
    if (config.verbose) { drvPrint("Turning Reverse\n"); }
}

void DRV8214::brakeMotor(bool initial_config) {
    enableHbridge();

    if (config.control_mode == PWM) {
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, true);
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, true);
    } else {
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, false);
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, false);
    }

    if (config.verbose && !initial_config) { drvPrint("Braking Motor\n"); }
}

void DRV8214::coastMotor() {
    enableHbridge();

    if (config.control_mode == PWM) {
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_EN_IN1, false);
        drv8214_i2c_modify_register(address, DRV8214_CONFIG4, CONFIG4_I2C_PH_IN2, false);
    } else {
        drvPrint("PH/EN mode does not support coast (Hi-Z) while awake.\n");
    }

    if (config.verbose) { drvPrint("Coasting Motor\n"); }
}

void DRV8214::turnXRipples(uint16_t ripples_target, bool stops, bool direction,
                           uint16_t speed, float voltage, float requested_current) {
    setRippleCountThreshold(ripples_target);
    resetRippleCounter();
    if (stops != config.bridge_behavior_thr_reached) {
        setBridgeBehaviorThresholdReached(stops);
    }
    if (direction) {
        turnForward(speed, voltage, requested_current);
    } else {
        turnReverse(speed, voltage, requested_current);
    }
}

void DRV8214::turnXRevolutions(uint16_t revolutions_target, bool stops, bool direction,
                               uint16_t speed, float voltage, float requested_current) {
    uint16_t ripples_target = revolutions_target * ripples_per_revolution * motor_reduction_ratio;
    turnXRipples(ripples_target, stops, direction, speed, voltage, requested_current);
}

/* ──────────────────── Debug / Diagnostics ──────────────────── */

void DRV8214::printMotorConfig(bool initial_config) {
    char buffer[256];

    if (initial_config) {
        snprintf(buffer, sizeof(buffer), "----- Initialized driver %d -----\n", driver_ID);
    } else {
        snprintf(buffer, sizeof(buffer), "DRV8214 Driver %d\n", driver_ID);
    }
    drvPrint(buffer);

    snprintf(buffer, sizeof(buffer),
             "Address: 0x%02X | Rsense: %d Ohm | Ripples/rotor: %d | Ripples/shaft: %d\n",
             address, Ripropri, ripples_per_revolution,
             ripples_per_revolution * motor_reduction_ratio);
    drvPrint(buffer);

    snprintf(buffer, sizeof(buffer),
             "OVP: %s | Stall: %s | I2C: %s | Mode: %s",
             config.ovp_enabled ? "On" : "Off",
             config.stall_enabled ? "On" : "Off",
             config.I2CControlled ? "Yes" : "No",
             (config.control_mode == PWM) ? "PWM" : "PH/EN");
    drvPrint(buffer);

    drvPrint(" | Regulation: ");
    switch (config.regulation_mode) {
        case CURRENT_FIXED:  drvPrint("CURRENT_FIXED\n");  break;
        case CURRENT_CYCLES: drvPrint("CURRENT_CYCLES\n"); break;
        case SPEED:          drvPrint("SPEED\n");           break;
        case VOLTAGE:        drvPrint("VOLTAGE\n");         break;
    }

    snprintf(buffer, sizeof(buffer),
             "Vref: %.3f V | IMODE: %d | VRange: %s\n",
             config.Vref, config.current_reg_mode,
             config.voltage_range ? "0-3.92V" : "0-15.7V");
    drvPrint(buffer);

    snprintf(buffer, sizeof(buffer),
             "Stall: %s | Bridge@thr: %s\n",
             config.stall_behavior ? "Keep driving" : "Disable outputs",
             config.bridge_behavior_thr_reached ? "Hi-Z" : "Keep driving");
    drvPrint(buffer);

    snprintf(buffer, sizeof(buffer),
             "Inrush: %d ms | INV_R: %d | INV_R_SCALE: %d\n",
             config.inrush_duration, config.inv_r, config.inv_r_scale);
    drvPrint(buffer);

    snprintf(buffer, sizeof(buffer), "KMC: %d | KMC_SCALE: %d\n",
             config.kmc, config.kmc_scale);
    drvPrint(buffer);
}

void DRV8214::drvPrint(const char* msg) {
#ifdef DRV8214_PLATFORM_ARDUINO
    if (_debugPort) {
        _debugPort->print(msg);
    }
#elif defined(DRV8214_PLATFORM_STM32)
    printf("%s", msg);
#endif
}

void DRV8214::printFaultStatus() {
    char buffer[128];
    uint8_t faultReg = drv8214_i2c_read_register(address, DRV8214_FAULT);

    snprintf(buffer, sizeof(buffer), "DRV8214 #%d FAULT [0x%02X]:", driver_ID, faultReg);
    drvPrint(buffer);

    if (faultReg & FAULT_FAULT)    { drvPrint(" FAULT"); }
    if (faultReg & FAULT_STALL)    { drvPrint(" STALL"); }
    if (faultReg & FAULT_OCP)      { drvPrint(" OCP"); }
    if (faultReg & FAULT_OVP)      { drvPrint(" OVP"); }
    if (faultReg & FAULT_TSD)      { drvPrint(" TSD"); }
    if (faultReg & FAULT_NPOR)     { drvPrint(" NPOR"); }
    if (faultReg & FAULT_CNT_DONE) { drvPrint(" CNT_DONE"); }
    if (!(faultReg & 0xFD))        { drvPrint(" OK"); }
    drvPrint("\n");
}

#ifdef DRV8214_PLATFORM_ARDUINO
void DRV8214::setDebugStream(Stream* debugPort) {
    _debugPort = debugPort;
}
#endif
