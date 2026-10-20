#pragma once

// Must match .agents/rules/hardware_map.md (ground truth) and docs/wiring.jpg

// I2C – BMI270 IMU
#define PIN_SDA     21
#define PIN_SCL     22

// ESC signal outputs – Betaflight motor layout (Quad X, viewed from above)
#define PIN_MOTOR_1 13  // Rear-right
#define PIN_MOTOR_2 14  // Front-right (outputs PWM during boot – test with props off)
#define PIN_MOTOR_3 27  // Rear-left
#define PIN_MOTOR_4 26  // Front-left

// RC Input – planned CRSF on UART2 (not wired yet)
#define PIN_RC_RX   16
#define PIN_RC_TX   17

// SPI (VSPI) – DPS310 barometer
#define PIN_SPI_MOSI 23
#define PIN_SPI_MISO 19
#define PIN_SPI_SCK  18
#define PIN_BARO_CS  5

// Battery sensing – not wired yet. Must be ADC1 pins (32–39): ADC2 is unusable while WiFi is on.
// #define PIN_VBAT_ADC  TODO  // via voltage divider, never direct
// #define PIN_CURR_ADC  TODO

// Status LED (onboard)
#define PIN_STATUS_LED 2
