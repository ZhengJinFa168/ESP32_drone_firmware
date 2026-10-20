# Coding Conventions

The AI agent must strictly follow these rules when writing code for this project.

## Embedded / ESP32 Specifics
- **No Blocking Code**: Never use `delay()`. Always use `vTaskDelay()` or non-blocking timer checks (e.g. `millis()` diffs).
- **Memory Management**: Do not use `malloc()` or `new` inside the main flight loop to prevent heap fragmentation. Use static allocation (like `std::array`) or pass pre-allocated contexts. (See `EventService` and `ComplementaryFilter` for examples of zero-allocation architectures).
- **Logging**: Use standard logging rather than `Serial.print()` to keep outputs uniform.

## Style Guide
- **Classes/Structs**: `PascalCase` (e.g., `ComplementaryFilter`, `EventService`, `HeartbeatModule`)
- **Methods/Functions**: `camelCase` (e.g., `setup()`, `publish()`, `getRoll()`)
- **Class Member Variables**: Prefix with an underscore `_` and use `camelCase` (e.g., `_roll`, `_pitch`, `_listeners`, `_alpha`)
- **Local Variables**: `camelCase` (e.g., `rollAcc`, `accelValid`)
- **Constants/Macros**: `UPPER_SNAKE_CASE` (e.g., `MAX_LISTENERS_PER_EVENT`, `RAD_TO_DEG`)
- **Pointers/References**: Attach the modifier to the type, not the variable name (e.g., `void* context`, `EventService& instance()`)
- **Indentation**: 4 spaces

## File Structure
- Headers (`.h`) must use `#pragma once` (preferred over `#ifndef` include guards).
- Utilize standard `<cmath>` and `<array>` instead of raw C math/arrays when appropriate, but avoid `<vector>` and `<string>` for real-time loops.

## Safety & Reliability
- **Safety Comments**: Always add detailed inline comments on any safety-relevant logic (e.g., motor outputs, failsafes, PID math).
- **Loud Failures**: Prefer loud failures (e.g., immediate disarming, error logging, or system halt) over silent clamping when an invalid state is detected.
- **Actuator Risk Callouts**: Include a bold "**Risk Callout**" as the last line of any markdown response that proposes changes to actuator-related code (motors, ESCs).
- **Dependencies**: Always pin exact library versions in `platformio.ini` to prevent upstream breaking changes.
