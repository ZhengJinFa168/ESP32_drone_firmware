# Drone Firmware

Modular drone flight controller firmware built with PlatformIO and Arduino framework.

## Structure
- `src/core/` – Module manager, event bus
- `src/modules/` – Sensors, actuators, communication, flight logic
- `src/utils/` – Shared header-only utilities
- `test/` – Unit and integration tests
- `docs/` – Architecture, wiring, tuning guides

## Getting Started
1. Install [PlatformIO](https://platformio.org/)
2. `pio run` to build
3. `pio run -t upload` to flash
4. See `docs/pid_tuning_guide.md` for tuning instructions
