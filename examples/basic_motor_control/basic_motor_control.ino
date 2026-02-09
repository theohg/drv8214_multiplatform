/**
 * @file basic_motor_control.ino
 * @brief Basic motor control example for the DRV8214 library.
 *
 * Demonstrates forward/reverse driving, ripple-based positioning,
 * and status readback on Arduino / ESP32.
 */

#include <Wire.h>
#include <drv8214.h>

/* ── Adapt these to your hardware ── */
#define I2C_ADDR          DRV8214_I2C_ADDR_00  // 0x30 (7-bit)
#define SENSE_RESISTOR    1000   // IPROPI resistor [Ohms]
#define RIPPLES_PER_REV   6      // Current ripples per rotor revolution
#define MOTOR_RESISTANCE  12     // Winding resistance [Ohms]
#define REDUCTION_RATIO   298    // Gearbox ratio
#define MAX_RPM           100    // Max motor RPM

DRV8214 motor(I2C_ADDR, 0, SENSE_RESISTOR, RIPPLES_PER_REV,
              MOTOR_RESISTANCE, REDUCTION_RATIO, MAX_RPM);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    motor.setDebugStream(&Serial);

    // Configure: PWM mode, speed regulation, stall detection on
    DRV8214_Config cfg;
    cfg.verbose = true;
    motor.init(cfg);
}

void loop() {
    // Forward for 5000 ripples, auto-stop at threshold
    motor.turnXRipples(5000, true, true);
    delay(3000);

    // Read status
    Serial.print("Speed: ");
    Serial.print(motor.getMotorSpeedShaftRPM());
    Serial.println(" RPM");

    Serial.print("Current: ");
    Serial.print(motor.getMotorCurrent());
    Serial.println(" A");

    motor.printFaultStatus();

    // Reverse for 2 full output shaft revolutions
    motor.turnXRevolutions(2, true, false);
    delay(3000);

    motor.brakeMotor();
    delay(2000);
}
