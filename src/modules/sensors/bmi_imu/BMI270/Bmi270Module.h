#pragma once
#include "core/Module.h"

/*
 * BMI270 IMU Module
 * 
 * Hardware Connection (I2C):
 * - 3.3V : ESP32 3.3V
 * - GND  : ESP32 GND
 * - SCL  : ESP32 GPIO 22 (PIN_SCL)
 * - SDA  : ESP32 GPIO 21 (PIN_SDA)
 * - INT  : Not currently used (can be used for hardware interrupts)
 * - GND  : ESP32 GND
 * 
 * Library Information:
 * We use the `SparkFun_BMI270_Arduino_Library` because the BMI270 requires a 
 * complex initialization sequence. Unlike simpler IMUs, the BMI270 loads an 
 * 8KB configuration file into its internal RAM during startup to configure its 
 * DSP (Digital Signal Processor). The SparkFun library abstracts away this 
 * initialization process and also automatically scales the raw register values 
 * into standard physical units (g for accelerometer, degrees/second for gyroscope).
 *
 * Health (see ImuHealthMonitor):
 * - Every read result goes through ImuHealthMonitor; changes are published as
 *   SENSOR_FAULT (arming blocks / disarms on it).
 * - Data from an unhealthy sensor is never published as IMU_RAW.
 * - Every IMU_REG_CHECK_MS the power-control register is read: 0 means the sensor
 *   reset (e.g. a power glitch) and lost its configuration.
 * - While faulty, recovery (I2C bus reset + re-init) is attempted at most every
 *   IMU_RECOVERY_RETRY_MS. Re-init uploads 8 KB of config and blocks the loop for a
 *   few hundred ms; acceptable because a fault has already disarmed the drone.
 * NOTE: register check, bus recovery and re-init are not yet tested on hardware.
 */
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Wire.h>
#include "config/SensorConfig.h"
#include "config/pins.h"
#include "modules/sensors/bmi_imu/ImuHealthMonitor.h"

class Bmi270Module : public Module {
private:
    // SparkFun's BMI270() constructor leaves its internal bmi2_dev struct uninitialized,
    // and bmi270_init() only sets config_file_ptr when it is null. A stack or heap
    // instance therefore uploads garbage config (init fails) or crashes.
    // Function-local statics are zero-initialized, so the driver lives here.
    // Consequence: one BMI270 per firmware (all modules share this instance).
    static BMI270& driver() {
        static BMI270 imu;
        return imu;
    }

    BMI270& _imu;
    unsigned long _lastReadTimeUs = 0;
    unsigned long _readIntervalUs = 2500; // 400 Hz default read rate

    // Hardware settings
    uint8_t _i2cAddress;
    TwoWire& _wire;

    // Health
    ImuHealthMonitor _health;
    bool _initialized = false;
    uint32_t _lastRegCheckMs = 0;

    static uint32_t nowMs() { return static_cast<uint32_t>(millis()); }

    bool initSensor() {
        if (_imu.beginI2C(_i2cAddress, _wire) != BMI2_OK) return false;

        // Configure ODR and Power modes for low latency / continuous data
        _imu.setAccelODR(BMI2_ACC_ODR_400HZ);
        _imu.setGyroODR(BMI2_GYR_ODR_400HZ);

        // Disable advanced power save to get continuous high-speed data
        _imu.disableAdvancedPowerSave();
        return true;
    }

    // Read one register directly (the library's register access is private). -1 on bus error.
    int readRegister(uint8_t reg) {
        _wire.beginTransmission(_i2cAddress);
        _wire.write(reg);
        if (_wire.endTransmission(false) != 0) return -1;
        if (_wire.requestFrom(_i2cAddress, static_cast<uint8_t>(1)) != 1) return -1;
        return _wire.read();
    }

    // Free a sensor that holds SDA low mid-transfer: clock SCL up to 9 times, then
    // send a STOP. The microsecond waits only happen during recovery.
    void recoverBus() {
        _wire.end();
        pinMode(PIN_SDA, INPUT_PULLUP);
        pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
        digitalWrite(PIN_SCL, HIGH);
        for (int i = 0; i < 9 && digitalRead(PIN_SDA) == LOW; i++) {
            digitalWrite(PIN_SCL, LOW);
            delayMicroseconds(5);
            digitalWrite(PIN_SCL, HIGH);
            delayMicroseconds(5);
        }
        // STOP condition: SDA rises while SCL is high
        pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
        digitalWrite(PIN_SDA, LOW);
        delayMicroseconds(5);
        digitalWrite(PIN_SCL, HIGH);
        delayMicroseconds(5);
        digitalWrite(PIN_SDA, HIGH);
        delayMicroseconds(5);

        beginBus();
    }

    void beginBus() {
        _wire.begin(PIN_SDA, PIN_SCL);
        _wire.setClock(400000); // 400kHz fast mode
    }

    void attemptRecovery(uint32_t now) {
        _health.onRecoveryAttempt(now);
        Serial.println("BMI270: recovery attempt (I2C bus reset + re-init, loop blocked briefly)...");
        recoverBus();
        _initialized = initSensor();
        if (_initialized) {
            Serial.println("BMI270: re-initialized, fault clears after healthy data");
        } else {
            Serial.println("BMI270: recovery failed, will retry");
            report(_health.onInitResult(false));
        }
        _lastReadTimeUs = micros();
        _lastRegCheckMs = nowMs();
    }

    // SAFETY: every health change is published, so arming can block or disarm.
    void report(ImuHealthMonitor::Change change) {
        if (change == ImuHealthMonitor::Change::NONE) return;

        if (change == ImuHealthMonitor::Change::FAULT_RAISED) {
            Serial.print("SENSOR FAULT: IMU, reason: ");
            Serial.print(sensorFaultReasonName(_health.reason()));
            Serial.print(", fault count: ");
            Serial.println(_health.faultCount());
        } else {
            Serial.println("SENSOR OK: IMU fault cleared");
        }

        SensorFaultData fault{SensorId::IMU, _health.isFaulty(), _health.reason()};
        publish(EventType::SENSOR_FAULT, &fault);
    }

    void checkPowerRegister(uint32_t now) {
        if (now - _lastRegCheckMs < IMU_REG_CHECK_MS) return;
        _lastRegCheckMs = now;

        const int pwrCtrl = readRegister(BMI2_PWR_CTRL_ADDR);
        if (pwrCtrl < 0) {
            report(_health.onReadError());
        } else if (pwrCtrl == 0) {
            // Accel and gyro disabled: the sensor reset and lost its configuration
            report(_health.onResetDetected());
        }
    }

public:
    explicit Bmi270Module(uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR, TwoWire& wire = Wire, unsigned long readIntervalUs = 2500)
        : Module("BMI270_IMU"), _imu(driver()), _readIntervalUs(readIntervalUs), _i2cAddress(i2cAddress), _wire(wire) {}

    void setup() override {
        beginBus();
        Serial.println("Initializing BMI270...");

        _initialized = initSensor();
        if (_initialized) {
            Serial.println("BMI270 initialized successfully.");
        } else {
            // Stay enabled: the module keeps retrying and reports the fault loudly
            Serial.println("ERROR: BMI270 init failed (no response or wrong chip ID). Will retry.");
            _health.onRecoveryAttempt(nowMs());
            report(_health.onInitResult(false));
        }

        // SAFETY: an unverified mounting alignment can flip the drone at takeoff.
        if (!IMU_ALIGNMENT_VERIFIED) {
            Serial.println("WARNING: IMU alignment NOT verified (SensorConfig.h) - arming will be blocked.");
        }

        _lastReadTimeUs = micros();
        _lastRegCheckMs = nowMs();
    }

    void loop() override {
        const uint32_t now = nowMs();
        if (_health.shouldAttemptRecovery(now)) {
            attemptRecovery(now);
            return;
        }
        if (!_initialized) return;

        unsigned long nowUs = micros();
        if (nowUs - _lastReadTimeUs < _readIntervalUs) return;
        _lastReadTimeUs = nowUs;

        checkPowerRegister(now);

        if (_imu.getSensorData() != BMI2_OK) {
            report(_health.onReadError());
            return;
        }

        float acc[3] = {_imu.data.accelX, _imu.data.accelY, _imu.data.accelZ};
        float gyro[3] = {_imu.data.gyroX, _imu.data.gyroY, _imu.data.gyroZ};
        report(_health.onSample(acc, gyro));

        // SAFETY: never publish data from an unhealthy sensor, and never publish an
        // all-zero sample (it only appears right after init or after a reset).
        if (_health.isFaulty() || ImuHealthMonitor::isAllZero(acc, gyro)) return;

        // Rotate sensor axes into the body frame (FLU) so every
        // downstream module is independent of how the chip is mounted.
        applyAlignment(IMU_ALIGNMENT, acc, acc);
        applyAlignment(IMU_ALIGNMENT, gyro, gyro);

        ImuData data{acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]};

        // Publish IMU_RAW event (body frame)
        publish(EventType::IMU_RAW, &data);
    }

    bool isHealthy() const { return !_health.isFaulty(); }
};
