# ESC Module

Controls up to 4 ESCs via PWM.

## Wiring
Connect each ESC signal wire to the motor pins defined in `config/pins.h`.
Power ESCs from the battery bus (not from the microcontroller 3.3V rail).

## Configuration
Adjust `ESC_MIN_PWM_US` / `ESC_MAX_PWM_US` to match your ESC calibration.
