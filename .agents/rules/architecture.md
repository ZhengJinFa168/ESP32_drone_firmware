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
