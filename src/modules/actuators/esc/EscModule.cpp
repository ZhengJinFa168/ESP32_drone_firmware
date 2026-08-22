#include "EscModule.h"
#include "EscConfig.h"
#include "config/pins.h"
#include "core/eventService/EventService.h"
#include <Arduino.h>

void EscModule::setup() {
    // Configure PWM channels for each motor
    // TODO: ledcSetup + ledcAttachPin for each motor pin
}

void EscModule::loop() {
    // Motor outputs are written on demand via setMotors()
}

void EscModule::setMotors(float m1, float m2, float m3, float m4) {
    _throttle[0] = m1; _throttle[1] = m2;
    _throttle[2] = m3; _throttle[3] = m4;
    // TODO: convert 0-1 throttle to PWM us and write via ledcWrite
}
