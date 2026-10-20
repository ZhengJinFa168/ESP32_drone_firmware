# Agent Session Notes

*This document acts as a memory bank across different AI agent sessions. Ask the agent to update this at the end of a session, and read it at the beginning of a new one.*

## Current State
- **Phase**: Firmware Architecture Setup & Sensor Integration
- **Working Components**: Module base class, EventService (Pub/Sub), Complementary Filter attitude estimator, BMI270 IMU module (I2C), WebSocket RC input module (builds; not yet tested on hardware).
- **Hardware**: Fully mapped in `.agents/rules/hardware_map.md` (ground truth) with diagram `docs/wiring.jpg`. `src/config/pins.h` matches it.
- **Current Blockers**: None.
- **Open Decisions**: ESC protocol (DShot300 recommended), motor KV, ADC pins for VBAT/CURR (must be G32–G39), BMI270 I2C address (expected 0x68), motor spin directions.
- **Known Issues / Next Up**:
  - RC input values are not range-checked (per conventions: reject + log, don't clamp).
  - RC queue uses `xQueueCreate` (heap); switch to `xQueueCreateStatic` to satisfy the no-dynamic-allocation rule.
  - Code uses `Serial.print` instead of standard logging (`log_e`/`log_i`), against conventions.
  - `platformio.ini` uses `^` version ranges; conventions require exact pinned versions.
  - `App::loop()` uses `delay(1)` (forbidden by conventions); plan is an IMU-interrupt-driven flight task pinned to core 1.
  - `Bmi270Module` polls in `loop()` instead of using the INT pin; ignores `PIN_SDA`/`PIN_SCL`; `-Wreorder` warning.
  - `EscModule` is a stub and has no constructor passing a name to `Module`.
  - Complementary filter uses a fixed `alpha` independent of `dt`; no gyro bias calibration.
  - `send_P` deprecation warning in `WebSocketRcProvider.h`.

## Session Log
*Newest first. Prepend each new session entry directly below this line.*

### [2026-10-20] - Repo Review, RC Thread Safety, Hardware Map
- **Repo review**: Native unit tests pass (21/21); firmware builds for `nodemcu-32s` (RAM 13.6%, Flash 67.2% with WiFi + async web server).
- **Repo cleanup**:
  - Untracked 20 committed `.DS_Store` files (already in `.gitignore`).
  - Removed `search_result.json` (a saved PlatformIO registry HTML page).
  - Removed `.github/` from `.gitignore` so new CI workflows get tracked.
  - Side effect: placeholder folders that only held `.DS_Store` (`motors/`, `lora/`, `micro_ros/`, `control_loop/`, `flight_logic/`) no longer exist in fresh clones.
- **RC input data race fixed** (`WebSocketRcProvider.h`):
  - Context: the RC module itself (`RcInputModule.h`, `WebSocketRcProvider.h`, `WebUI.h`, ESPAsyncWebServer dependency, registration in `main.cpp`) came from an earlier, unlogged session and is still uncommitted. This session only changed how the provider hands data to the loop.
  - Problem: `onWsEvent` runs on the `async_tcp` task (priority 10, any core) while `getRcData` runs on `loopTask` (priority 1, core 1). They shared `currentData`/`newDataAvailable` unsynchronized → torn reads and lost packets.
  - Fix: replaced with a length-1 FreeRTOS "mailbox" queue. Writer uses `xQueueOverwrite` (never blocks); reader uses non-blocking `xQueueReceive`. If queue creation fails, RC reports no data and the failsafe triggers.
  - Discussed alternatives: `portMUX` spinlock, mutex, seqlock (candidate for future flight-loop → telemetry handoff), and why `volatile` / `std::atomic<struct>` don't solve it.
- **Hardware map** (`.agents/rules/hardware_map.md`): filled from the wiring diagram and parts list.
  - Board: AZ-Delivery ESP32 DevKitC 38-pin (WROOM-32). Core 0 = WiFi/telemetry, Core 1 = flight loop.
  - Power: 4S 1550 mAh LiPo; UBEC 5V → ESP32 5V/GND; sensors on 3V3.
  - ESC: JHEMCU EM40A 4-in-1 BLHeli_S. Motors (Betaflight layout): M1 rear-right G13, M2 front-right G14, M3 rear-left G27, M4 front-left G26.
  - IMU: BMI270 I2C SDA G21 / SCL G22. Barometer: DPS310 on VSPI (MOSI 23, MISO 19, SCK 18, CS 5).
  - Added "Pins to Avoid" section (flash pins, strapping pins, ADC2 unusable with WiFi, boot-glitch pins).
  - Note: the diagram's "Data line"/"Clock line" labels are swapped (G21 = SDA, G22 = SCL); the wiring is correct.
- **`pins.h`** synced to the hardware map: motor pins moved off strapping pin G12, `PIN_SPI_CS` → `PIN_BARO_CS`, added `PIN_RC_TX` and `PIN_STATUS_LED`, ADC pins left as TODO.
- Saved wiring diagram to `docs/wiring.jpg`.
- **Agent setup**:
  - `.gitignore` now ignores `.agents/*` except `.agents/rules/`, so the rules are versioned.
  - Added `CLAUDE.md` importing all `.agents/rules/*.md` files, plus build/test commands (`pio` lives at `~/.platformio/penv/bin/pio`).
- Nothing committed yet.

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

### [2026-09-11] - Workspace Setup
- Created `.agents/rules/` directory to hold agent instructions.
- Generated `hardware_map.md`, `architecture.md`, and `conventions.md`.
- Moved `workflow.md` to the rules folder so the agent automatically loads it.
