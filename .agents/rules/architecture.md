# Software Architecture

This document describes the high-level architecture of the ESP32 drone firmware. The AI agent MUST read and adhere to these architectural patterns.

## Core Concepts
- **Modules**: The system is highly modular. All subsystems (sensors, logic, actuators) inherit from the base `Module` class (`src/core/Module.h`). The central `App` class registers and manages them.
- **Event Service**: Modules communicate using a zero-allocation Publisher/Subscriber pattern via `EventService`. 
  - Modules must NOT tightly couple or call methods on each other directly.
  - Subscriptions use static C-style callbacks passing `this` as context to prevent heap allocation (e.g., `static void onEvent(void* ctx, const void* data)`).
  - Event types and POD (Plain Old Data) structs are defined centrally in `src/core/eventService/EventType.h`.

## Execution / Control Flow
- The architecture favors being **fully event-driven**.
- Instead of polling sensors inside a `loop()`, hardware sensor modules read data and `publish()` events (like `IMU_RAW`).
- Downstream modules (like `AttitudeEstimatorModule`) `subscribe()` to these events, compute their math inside the callback, and `publish()` subsequent events (like `ATTITUDE_UPDATED`).
- The `Module::loop()` function is available for periodic tasks (like a Heartbeat), but rapid control loops should chain off event callbacks to minimize latency.

## Body Frame Convention
- All modules downstream of the IMU work in the **body frame, FLU**: X forward, Y left, Z up (ROS REP-103).
  - Level and still: accelerometer reads `az = +1 g`.
  - Angles: roll + = right side down, pitch + = nose down, yaw + = nose left (counter-clockwise from above).
  - Rates: right-hand rule about each axis.
- Sensor modules rotate their readings into the body frame **before** publishing (`applyAlignment` in `src/utils/AxisAlignment.h`, configured in `src/config/SensorConfig.h`). No other module may depend on how a sensor is mounted.
- The mounting alignment is set and verified at the end of the build (see "End-of-Build Checklist" in `hardware_map.md`). Arming must be refused while `IMU_ALIGNMENT_VERIFIED` is false.

## Hardware Abstraction (Portability)
Goal: the microcontroller can be swapped (e.g. ESP32 → ESP32-S3, STM32, RP2040) by adding a platform implementation, without changing the logic modules.
1. **Logic modules never touch hardware.** Estimator, PID, mixer, arming and similar modules must not include `Arduino.h` or any chip/vendor header. They use only plain C++, events and the interfaces below. This also keeps them unit-testable in the native env.
2. **Hardware access lives only in driver modules** (IMU, ESC/motor output, RC providers, sensors) behind small interfaces (e.g. `IMotorOutput`, RC provider, IMU bus, clock, logger).
3. **Chip-specific calls live only in the platform layer** (`src/platform/<chip>/`): e.g. `xTaskCreatePinnedToCore`, RMT, LEDC, ADC rules, WiFi. Pin numbers stay in `src/config/pins.h`.
- **Selection at compile time**: one firmware build targets one chip, so pick implementations with type aliases or templates (like `CurrentRcProvider` and `AttitudeEstimatorModule<TFilter>`), not runtime factories. Virtual interfaces are allowed where a runtime choice is genuinely needed.
- **No heap for abstraction**: implementations are globals, `static` objects or class members, never created with `new`.
- Add an interface only when a second implementation is realistic; do not abstract speculatively.

## Event Types & Structs
When passing data, use the existing structs in `EventType.h`:
- `ImuData` (ax, ay, az, gx, gy, gz)
- `AttitudeData` (roll, pitch, yaw)
- `RcInputData` (throttle, roll, pitch, yaw, aux[])
- `MotorCommandData` (motors[4])
- System Events: `ARMED`, `DISARMED`, `MODE_CHANGED`, `HEARTBEAT`

## Expected AI Agent Behavior
When modifying or creating new features, the agent MUST:
1. Inherit from the standard `Module` class if adding a subsystem.
2. Never allocate memory dynamically (e.g., no `std::vector`, no `std::function` for callbacks).
3. Use the `EventService` to pass POD structs for all inter-module communication.
4. Favor event-driven callbacks over placing heavy computation in the `loop()` function.
5. Keep hardware and chip-specific code out of logic modules (see "Hardware Abstraction").
