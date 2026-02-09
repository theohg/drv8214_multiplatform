/**
 * @file fault_monitoring.ino
 * @brief Fault monitoring and voltage-mode example for the DRV8214 library.
 *
 * Drives the motor in voltage regulation mode while continuously
 * monitoring voltage, current, duty cycle, and fault flags.
 */

#include <Wire.h>
#include <drv8214.h>

/* ── Adapt these to your hardware ── */
#define I2C_ADDR          DRV8214_I2C_ADDR_00
#define SENSE_RESISTOR    1000
#define RIPPLES_PER_REV   6
#define MOTOR_RESISTANCE  12
#define REDUCTION_RATIO   298
#define MAX_RPM           100

DRV8214 motor(I2C_ADDR, 0, SENSE_RESISTOR, RIPPLES_PER_REV,
              MOTOR_RESISTANCE, REDUCTION_RATIO, MAX_RPM);

void setup() {
    Serial.begin(115200);
    Wire.begin();

    motor.setDebugStream(&Serial);

    DRV8214_Config cfg;
    cfg.regulation_mode = VOLTAGE;
    cfg.voltage_range   = true;   // 0..3.92 V range
    cfg.verbose         = true;
    motor.init(cfg);
}

void loop() {
    // Drive forward at 2 V
    motor.turnForward(0, 2.0f, 0);

    // Monitor for 5 seconds (10 x 500 ms)
    for (int i = 0; i < 10; i++) {
        Serial.println("--- Status ---");

        Serial.print("Voltage: ");
        Serial.print(motor.getMotorVoltage());
        Serial.println(" V");

        Serial.print("Current: ");
        Serial.print(motor.getMotorCurrent());
        Serial.println(" A");

        Serial.print("Duty: ");
        Serial.print(motor.getDutyCycle());
        Serial.println(" %");

        Serial.print("Ripples: ");
        Serial.println(motor.getRippleCount());

        // Check faults
        uint8_t fault = motor.getFaultStatus();
        if (fault & FAULT_FAULT) {
            motor.printFaultStatus();
            motor.resetFaultFlags();
        }

        delay(500);
    }

    motor.brakeMotor();
    delay(3000);
}
