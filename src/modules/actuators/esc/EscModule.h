#pragma once
#include "core/Module.h"

class EscModule : public Module {
public:
    void setup() override;
    void loop() override;
    void setMotors(float m1, float m2, float m3, float m4);

private:
    float _throttle[4] = {};
};
