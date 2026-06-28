#pragma once

#include "lib/subsystems/CanCoderInputs.h"

class CanCoderIO {
public:
    virtual ~CanCoderIO() = default;
    virtual void ReadInputs(CanCoderInputs& inputs) = 0;
    virtual void UpdateFrequency(double hz) = 0;
};
