# Hardware Map: ESP32 Drone Firmware

This document is the ground truth for hardware pinouts and peripheral configuration.
`src/config/pins.h` must match this file. If they disagree, this file wins — fix `pins.h`.

Wiring diagram: [`docs/wiring.jpg`](../../docs/wiring.jpg)
Known diagram errors (the wires themselves are correct):
- ESP32-side labels "Data line" / "Clock line" are swapped — G21 is SDA, G22 is SCL.
- The BMI270's two green "SDO" labels are wrong — the real module's pins 5 and 6 are INT and GND (the module image on the shop page differs from the actual board).

Legend: **[TO VERIFY]** = wired but not yet confirmed on hardware. **[TO DECIDE]** = design choice still open. **[NOT WIRED]** = planned, no physical connection yet.

## Microcontroller
- **Board**: AZ-Delivery ESP32 DevKitC, 38-pin, ESP32-WROOM-32 module (no PSRAM, so GPIO 16/17 are free)
- **Core Allocation**:
  - Core 0: WiFi stack, AsyncTCP / WebSocket RC, telemetry, logging
  - Core 1: Flight control loop (IMU → attitude estimator → PID → ESC), Arduino `loop()` for low-priority housekeeping

## Power
- **Battery**: 4S LiPo, 1550 mAh, 100C, XT60 (14.8 V nominal, 16.8 V full, do not discharge below ~3.5 V/cell under load)
- **ESC**: powered directly from the battery
- **BEC**: UBEC, 5.5–35 V in → 5 V out
  - BEC IN+ / IN− ← battery + / −
  - BEC OUT+ → ESP32 **5V** pin
  - BEC OUT− → ESP32 **GND** pin (bottom-left, next to 5V)
- **Sensors**: powered from ESP32 **3V3** pin (onboard LDO)
- **Common ground**: ESC GND, BEC OUT−, sensor GNDs and ESP32 GND are all joined. Required for the motor signal reference.
- **Caution**: avoid powering from USB and BEC at the same time unless the board has a diode isolating the 5V pin from USB VBUS **[TO VERIFY]**

## Motors / ESCs
- **ESC**: JHEMCU EM40A, 4-in-1, 40 A, BLHeli_S, 2–6S, 20×20 mm
- **Protocol**: **[TO DECIDE]** — supported: PWM, Oneshot125, Multishot, DShot150/300/600 (DShot on ESP32 via the RMT peripheral)
- **Motors**: iFlight XING-E Pro 2207, 2450 KV (4S-rated; matches the 4S battery)
- **Propellers**: **[TO DECIDE]** — 2207 2450 KV on 4S is the standard 5" setup (e.g. 5x4.3x3 tri-blade)
- **Layout**: Betaflight convention (Quad X, viewed from above, front at top)

| Motor | Position    | ESC pad | ESP32 GPIO | Spin direction |
|-------|-------------|---------|------------|----------------|
| M1    | Rear-right  | M1      | G13        | **[TO VERIFY]** |
| M2    | Front-right | M2      | G14        | **[TO VERIFY]** |
| M3    | Rear-left   | M3      | G27        | **[TO VERIFY]** |
| M4    | Front-left  | M4      | G26        | **[TO VERIFY]** |

- G14 outputs a PWM signal during boot. BLHeli_S requires a valid arming sequence, so this should not spin motors, but always test with props off.
- ESC `NC` pad: not connected.

## IMU / Gyroscope
- **Sensor**: BMI270 breakout (6-axis accel + gyro)
- **Bus**: I2C, 400 kHz
- **I2C SDA**: G21
- **I2C SCL**: G22
- **Module pinout** (actual board, in header order): 3.3V, GND, SCL, SDA, INT, GND
- **Power**: 3.3V ← 3V3, both GND pins ← GND
- **I2C Address**: 0x68 (confirmed 2026-10-21 by I2C scan; CHIP_ID reads 0x24). SDO is not exposed on the header, so the address is fixed by the module's onboard resistors. Code uses `BMI2_I2C_PRIM_ADDR`.
- **Mounting alignment**: `Align::CW0` (default, sensor axes = body axes) **[TO VERIFY]** — set in `src/config/SensorConfig.h` after mounting (see End-of-Build Checklist).
- **INT pin**: not connected yet. ESP32 GPIO **[TO DECIDE]** (planned: data-ready interrupt to drive the flight loop on core 1)

## Barometer
- **Sensor**: DPS310 breakout (pressure + temperature)
- **Bus**: SPI (VSPI)
- **MOSI → SDI**: G23
- **MISO ← SDO**: G19
- **SCK**: G18
- **CS**: G5
- **Power**: VIN ← 3V3, GND

## Battery Sensing **[NOT WIRED]**
- **Battery voltage (ESC VBAT pad)**: GPIO **[TO DECIDE]** — must be an ADC1 pin (G32–G39). Never connect directly: needs a voltage divider (e.g. 100 kΩ / 20 kΩ, ÷6 → 2.8 V at 16.8 V).
- **Current (ESC CURR pad)**: GPIO **[TO DECIDE]** — must be an ADC1 pin (G32–G39). Output voltage range and scale factor **[TO VERIFY]** from the EM40A manual before connecting.

## Receiver (RC)
- **Current**: WebSocket over WiFi (ESP32 soft AP, phone joystick UI) — no wiring
- **Planned**: CRSF on UART2
  - **UART RX**: G16 **[NOT WIRED]**
  - **UART TX**: G17 **[NOT WIRED]**

## Misc
- **Status LED**: G2 (onboard LED)

## End-of-Build Checklist
Everything is developed in the body frame (FLU) and Betaflight motor layout. These hardware facts are confirmed only after final assembly, and each one gates arming or first flight:
1. **IMU alignment** (gravity test on the mounted sensor, still, in Teleplot or serial output):
   - Nose pointing up → the sensor axis reading +1 g is the body forward axis (−1 g = it points backward).
   - Left side pointing up → the sensor axis reading +1 g is the body left axis.
   - Flat, chip side up/down → which sensor axis reads ±1 g is body up.
   - Set `IMU_ALIGNMENT` in `src/config/SensorConfig.h`, re-run the test (body readings: nose up → `ax = +1`, left up → `ay = +1`, level → `az = +1`), then set `IMU_ALIGNMENT_VERIFIED = true`.
2. **Motor order and spin direction** — props off, one motor at a time; confirm against the Motors table.
3. **ESC protocol** — decide and configure (see Motors / ESCs).
4. **Battery sensing pins** — wire the VBAT divider and CURR, record the ADC1 pins.

## Pins to Avoid / Constraints
- **Flash pins — never use**: G6–G11 (CLK, SD0, SD1, SD2, SD3, CMD)
- **Strapping pins** (affect boot mode, use with care): G0, G2, G5, G12, G15
  - G12 must be LOW at boot, otherwise the module selects 1.8 V flash and fails to boot. Do not use for outputs pulled high.
- **ADC2 pins** (G0, G2, G4, G12–G15, G25–G27) cannot read analog values while WiFi is active. Use ADC1 (G32–G39) for all analog inputs.
- **Input-only pins**: G34, G35, G36 (SVP), G39 (SVN) — no output, no internal pull-ups. Good for ADC.
- **Boot glitches**: G1, G3, G5, G14, G15 output signals during boot.
