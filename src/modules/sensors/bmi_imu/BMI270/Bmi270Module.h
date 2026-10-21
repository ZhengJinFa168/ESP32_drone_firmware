#pragma once
#include "core/Module.h"

/*
 * BMI270 IMU Module
 * 
 * Hardware Connection (I2C):
 * - 3.3V : ESP32 3.3V
 * - GND  : ESP32 GND
 * - SCL  : ESP32 GPIO 22 (Default I2C SCL)
 * - SDA  : ESP32 GPIO 21 (Default I2C SDA)
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
 */
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Wire.h>
#include "config/SensorConfig.h"

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

public:
    explicit Bmi270Module(uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR, TwoWire& wire = Wire, unsigned long readIntervalUs = 2500)
        : Module("BMI270_IMU"), _imu(driver()), _readIntervalUs(readIntervalUs), _i2cAddress(i2cAddress), _wire(wire) {}

    void setup() override {
        // Initialize I2C if not already initialized
        _wire.begin();
        _wire.setClock(400000); // 400kHz fast mode

        Serial.println("Initializing BMI270...");
        
        // Attempt to initialize BMI270
        if (_imu.beginI2C(_i2cAddress, _wire) != BMI2_OK) {
            Serial.println("Error: BMI270 not found. Please check wiring.");
            disable(); // Disable module if init fails
            return;
        }
        
        Serial.println("BMI270 initialized successfully.");

        // SAFETY: an unverified mounting alignment can flip the drone at takeoff.
        if (!IMU_ALIGNMENT_VERIFIED) {
            Serial.println("WARNING: IMU alignment NOT verified (SensorConfig.h) - arming will be blocked.");
        }

        // Configure ODR and Power modes for low latency / continuous data
        _imu.setAccelODR(BMI2_ACC_ODR_400HZ);
        _imu.setGyroODR(BMI2_GYR_ODR_400HZ);
        
        // Disable advanced power save to get continuous high-speed data
        _imu.disableAdvancedPowerSave();

        _lastReadTimeUs = micros();
    }

    void loop() override {
        if (!isEnabled()) return;

        unsigned long now = micros();
        if (now - _lastReadTimeUs >= _readIntervalUs) {
            _lastReadTimeUs = now;

            // Read sensor data
            if (_imu.getSensorData() == BMI2_OK) {
                // Rotate sensor axes into the body frame (FLU) so every
                // downstream module is independent of how the chip is mounted.
                float acc[3] = {_imu.data.accelX, _imu.data.accelY, _imu.data.accelZ};
                float gyro[3] = {_imu.data.gyroX, _imu.data.gyroY, _imu.data.gyroZ};
                applyAlignment(IMU_ALIGNMENT, acc, acc);
                applyAlignment(IMU_ALIGNMENT, gyro, gyro);

                ImuData data{acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]};

                // Publish IMU_RAW event (body frame)
                publish(EventType::IMU_RAW, &data);
            }
        }
    }
};

