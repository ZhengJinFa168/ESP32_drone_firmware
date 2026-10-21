# ESP32 Drone Firmware

Project rules live in `.agents/rules/` and are shared with other AI agents. Follow them exactly.

@.agents/rules/workflow.md
@.agents/rules/architecture.md
@.agents/rules/conventions.md
@.agents/rules/hardware_map.md

## Session notes
At the start of a session, read `NOTES.md` for the current project state. At the end, prepend a session entry to its log when asked.

## Build & test
PlatformIO is not on PATH; use `~/.platformio/penv/bin/pio`.
- Unit tests (native): `~/.platformio/penv/bin/pio test -e native`
- Firmware build: `~/.platformio/penv/bin/pio run -e nodemcu-32s`
- Hardware tests (needs the board connected): `~/.platformio/penv/bin/pio test -e nodemcu-32s`
- `default_envs = nodemcu-32s`, so plain `pio run` / `pio test` (and the VS Code buttons) target the firmware env
