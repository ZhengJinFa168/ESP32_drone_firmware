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

class Bmi270Module : public Module {
private:
    BMI270 _imu;
    unsigned long _lastReadTimeUs = 0;
    unsigned long _readIntervalUs = 2500; // 400 Hz default read rate

    // Hardware settings
    uint8_t _i2cAddress;
    TwoWire& _wire;

public:
    explicit Bmi270Module(uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR, TwoWire& wire = Wire, unsigned long readIntervalUs = 2500)
        : Module("BMI270_IMU"), _i2cAddress(i2cAddress), _wire(wire), _readIntervalUs(readIntervalUs) {}

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
                ImuData data;
                data.ax = _imu.data.accelX;
                data.ay = _imu.data.accelY;
                data.az = _imu.data.accelZ;
                data.gx = _imu.data.gyroX;
                data.gy = _imu.data.gyroY;
                data.gz = _imu.data.gyroZ;
                
                // Publish IMU_RAW event
                publish(EventType::IMU_RAW, &data);
            }
        }
    }
};

