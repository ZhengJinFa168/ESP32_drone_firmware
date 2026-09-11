# Agent Session Notes

*This document acts as a memory bank across different AI agent sessions. Ask the agent to update this at the end of a session, and read it at the beginning of a new one.*

## Current State
- **Phase**: Firmware Architecture Setup & Sensor Integration
- **Working Components**: Module base class, EventService (Pub/Sub), Complementary Filter attitude estimator, BMI270 IMU module (I2C).
- **Current Blockers**: None. Hardware pins partially defined (I2C uses default ESP32 pins 21/22).

## Session Log

### [2026-09-11] - Workspace Setup
- Created `.agents/rules/` directory to hold agent instructions.
- Generated `hardware_map.md`, `architecture.md`, and `conventions.md`.
- Moved `workflow.md` to the rules folder so the agent automatically loads it.

### [2026-09-11] - BMI270 IMU Implementation
- Identified the BMI270 IMU module as an I2C device (3.3V, GND, SCL, SDA, INT, GND).
- Added `sparkfun/SparkFun BMI270 Arduino Library@^1.0.3` to `platformio.ini` to handle the complex 8KB DSP initialization sequence and scale raw data to standard units (`g` and `deg/s`).
- Implemented `Bmi270Module` (`src/modules/sensors/bmi_imu/BMI270/Bmi270Module.h`), derived from `Module`.
  - Added initialization logic (`Wire.begin()` using default ESP32 pins GPIO 21 and 22, 400kHz).
  - Set ODR to 400Hz and disabled advanced power saving for continuous low-latency polling.
  - Added continuous polling loop to publish `IMU_RAW` events containing `ax, ay, az` and `gx, gy, gz`.
  - Added descriptive comments outlining the hardware pinout and reasoning behind using the library.
- Registered `Bmi270Module` in `main.cpp`.
- Added an integration test (`test/integration/test_bmi270/test_bmi270.cpp`) to verify I2C initialization and `IMU_RAW` event publishing on the real hardware.
- Successfully compiled the firmware with PlatformIO for the `nodemcu-32s` environment.

---
*(End of session. Ask the agent to prepend the next log here).*

