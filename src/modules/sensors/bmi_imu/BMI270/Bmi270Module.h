#pragma once
#include "core/Module.h"

/*
 * BMI270 IMU Module
 * 
 * Hardware Connection (I2C):
 * - 3.3V : ESP32 GPIO 4 (PIN_IMU_POWER, switched power, see "Power cycle" below)
 * - GND  : ESP32 GND
 * - SCL  : ESP32 GPIO 22 (PIN_SCL)
 * - SDA  : ESP32 GPIO 21 (PIN_SDA)
 * - INT  : ESP32 GPIO 32 (PIN_IMU_INT), wired but not used yet (Stage 2 flight task)
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
 * - While faulty, recovery (power cycle + re-init) is attempted at most every
 *   IMU_RECOVERY_RETRY_MS. Re-init uploads 8 KB of config and blocks the loop for
 *   ~300 ms (measured 296 ms); acceptable because a fault has already disarmed the
 *   drone. Stage 2 must move this out of the flight task.
 *
 * Power cycle (see ImuPowerSequencer):
 * - The module is powered from PIN_IMU_POWER. Every boot and every recovery attempt
 *   switches it off for IMU_POWER_OFF_MS, then on, then initializes it. A BMI270 that
 *   was partially powered during a dip comes back stuck (CHIP_ID 0x20) and only a real
 *   power-off clears it (bench-verified 2026-11-06).
 * - While off, SDA/SCL are driven LOW and INT has no pull-up: a high line back-powers
 *   the sensor through its pins and it never gets a clean power-on reset.
 * - The waits are non-blocking (millis()); only the init itself blocks.
 *
 * Gyro calibration (see GyroCalibrator):
 * - At boot, healthy body-frame samples go to the calibrator until it has averaged
 *   GYRO_CAL_SAMPLES still samples; then GYRO_CALIBRATION is published once.
 * - SAFETY: no IMU_RAW is published before that, so the estimator never integrates
 *   an uncorrected gyro. Afterwards the bias is subtracted from every sample.
 * - A sensor fault during calibration restarts it. The bias is kept across re-init.
 */
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Wire.h>
#include "core/Log.h"
#include "config/SensorConfig.h"
#include "config/pins.h"
#include "modules/sensors/bmi_imu/ImuHealthMonitor.h"
#include "modules/sensors/bmi_imu/GyroCalibrator.h"
#include "modules/sensors/bmi_imu/ImuPowerSequencer.h"

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

    // Power cycle (boot and recovery). The first init runs in loop(), so its result is
    // published from loop() too (never publish from setup()).
    ImuPowerSequencer _power;
    static constexpr uint8_t EXPECTED_CHIP_ID = 0x24;  // BMI270; 0x20 = stuck after a power dip

    // Gyro calibration
    GyroCalibrator _gyroCal;
    uint32_t _lastCalLogMs = 0;
    uint32_t _calRestartsLogged = 0;
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

    void beginBus() {
        _wire.begin(PIN_SDA, PIN_SCL);
        _wire.setClock(400000); // 400kHz fast mode
    }

    // Sensor off. SDA/SCL LOW and INT without pull-up: a high line would back-power the
    // sensor through its pins and it would come back stuck.
    void cutPower() {
        _wire.end();
        pinMode(PIN_SDA, OUTPUT);
        digitalWrite(PIN_SDA, LOW);
        pinMode(PIN_SCL, OUTPUT);
        digitalWrite(PIN_SCL, LOW);
        pinMode(PIN_IMU_INT, INPUT);
        digitalWrite(PIN_IMU_POWER, LOW);
    }

    // Sensor on. The lines are released first; I2C starts after the settle time.
    void restorePower() {
        pinMode(PIN_SDA, INPUT);
        pinMode(PIN_SCL, INPUT);
        digitalWrite(PIN_IMU_POWER, HIGH);
    }

    void startPowerCycle(uint32_t now) {
        if (_power.start(now) != ImuPowerSequencer::Action::CUT_POWER) return;
        _initialized = false;  // No reads until the sensor is initialized again
        cutPower();
    }

    // Runs the power cycle. Returns true while it is in progress (sensor must not be read).
    bool runPowerCycle(uint32_t now) {
        if (!_power.busy()) return false;
        switch (_power.update(now)) {
            case ImuPowerSequencer::Action::RESTORE_POWER: restorePower(); break;
            case ImuPowerSequencer::Action::INIT_SENSOR:   initAfterPowerUp(); break;
            default: break;
        }
        return true;
    }

    // Blocks ~300 ms (8 KB config upload).
    void initAfterPowerUp() {
        beginBus();
        _initialized = initSensor();
        if (_initialized) {
            if (_health.isFaulty()) {
                LOG_I("BMI270", "re-initialized after power cycle, fault clears after healthy data");
            } else {
                LOG_I("BMI270", "initialized");
            }
        } else {
            logInitFailure();
            report(_health.onInitResult(false));
        }
        _lastReadTimeUs = micros();
        _lastRegCheckMs = nowMs();
    }

    // Says why init failed: no answer (wiring/power) or a wrong chip ID (0x20 = stuck)
    void logInitFailure() {
        const int chipId = readRegister(BMI2_CHIP_ID_ADDR);
        if (chipId < 0) {
            LOG_E("BMI270", "init failed: no answer on I2C (wiring or power?), will retry");
        } else if (chipId != EXPECTED_CHIP_ID) {
            LOG_E("BMI270", "init failed: chip ID 0x%02X (expected 0x%02X, 0x20 = stuck), will retry",
                  chipId, EXPECTED_CHIP_ID);
        } else {
            LOG_E("BMI270", "init failed: chip ID OK but configuration failed, will retry");
        }
    }

    void attemptRecovery(uint32_t now) {
        _health.onRecoveryAttempt(now);
        LOG_W("BMI270", "recovery attempt: power cycle + re-init (loop blocked ~300 ms)...");
        startPowerCycle(now);
    }

    // SAFETY: every health change is published, so arming can block or disarm.
    void report(ImuHealthMonitor::Change change) {
        if (change == ImuHealthMonitor::Change::NONE) return;

        if (change == ImuHealthMonitor::Change::FAULT_RAISED) {
            LOG_E("BMI270", "SENSOR FAULT, reason: %s, fault count: %lu",
                  sensorFaultReasonName(_health.reason()), static_cast<unsigned long>(_health.faultCount()));
        } else {
            LOG_I("BMI270", "SENSOR OK, fault cleared");
        }

        // Samples from before the fault may be bad: never let them into the bias
        if (change == ImuHealthMonitor::Change::FAULT_RAISED) _gyroCal.restart();

        SensorFaultData fault{SensorId::IMU, _health.isFaulty(), _health.reason()};
        publish(EventType::SENSOR_FAULT, &fault);
    }

    void handleCalibration(GyroCalibrator::Result result, uint32_t now) {
        using Result = GyroCalibrator::Result;
        if (result == Result::COMPLETED) {
            LOG_I("BMI270", "gyro calibrated, bias (deg/s): x %.3f, y %.3f, z %.3f (restarts: %lu)",
                  _gyroCal.bias(0), _gyroCal.bias(1), _gyroCal.bias(2),
                  static_cast<unsigned long>(_gyroCal.restarts()));
            GyroCalibrationData cal{true, _gyroCal.bias(0), _gyroCal.bias(1), _gyroCal.bias(2)};
            publish(EventType::GYRO_CALIBRATION, &cal);
            return;
        }
        if (result != Result::RESTARTED_MOTION && result != Result::RESTARTED_ACCEL &&
            result != Result::RESTARTED_BIAS) {
            return;
        }
        // Restarts can happen every sample while the drone is handled: rate-limit the log
        if (now - _lastCalLogMs < GYRO_CAL_LOG_INTERVAL_MS) return;
        _lastCalLogMs = now;
        LOG_W("BMI270", "gyro calibration restarted (%s), %lu restarts since last report: keep the drone still",
              gyroCalResultName(result),
              static_cast<unsigned long>(_gyroCal.restarts() - _calRestartsLogged));
        _calRestartsLogged = _gyroCal.restarts();
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
        // INT never gets a pull-up (back-feed); the power pin starts LOW (sensor off)
        pinMode(PIN_IMU_INT, INPUT);
        pinMode(PIN_IMU_POWER, OUTPUT);

        // Clean start: power-cycle the sensor even after a plain ESP32 reset (the sensor
        // may be stuck from before). Init runs in loop() once the cycle completes; a
        // failure there is reported loudly and retried every IMU_RECOVERY_RETRY_MS.
        LOG_I("BMI270", "power-cycling the sensor, init follows in loop()...");
        const uint32_t now = nowMs();
        _health.onRecoveryAttempt(now);  // The first retry waits IMU_RECOVERY_RETRY_MS
        startPowerCycle(now);

        // SAFETY: an unverified mounting alignment can flip the drone at takeoff.
        if (!IMU_ALIGNMENT_VERIFIED) {
            LOG_W("BMI270", "IMU alignment NOT verified (SensorConfig.h), arming will be blocked");
        }
        LOG_I("BMI270", "gyro calibration starts with the first healthy samples: keep the drone still");
    }

    void loop() override {
        const uint32_t now = nowMs();
        if (runPowerCycle(now)) return;
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

        // SAFETY: nothing is published until the gyro bias is known
        if (!_gyroCal.isCalibrated()) {
            handleCalibration(_gyroCal.onSample(acc, gyro), now);
            return;
        }
        _gyroCal.apply(gyro);

        ImuData data{acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], static_cast<uint32_t>(nowUs)};

        // Publish IMU_RAW event (body frame)
        publish(EventType::IMU_RAW, &data);
    }

    bool isHealthy() const { return !_health.isFaulty(); }
    bool isGyroCalibrated() const { return _gyroCal.isCalibrated(); }
    bool isInitialized() const { return _initialized; }
    uint32_t powerCycles() const { return _power.cycles(); }
};
