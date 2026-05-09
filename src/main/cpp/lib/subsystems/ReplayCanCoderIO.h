#pragma once

#include "lib/subsystems/CanCoderIO.h"

class ReplayCanCoderIO : public CanCoderIO {
public:
    void ReadInputs(CanCoderInputs& inputs) override {}
    void UpdateFrequency(double hz) override {}
};
