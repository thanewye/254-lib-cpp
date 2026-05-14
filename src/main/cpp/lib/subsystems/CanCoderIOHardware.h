#pragma once

#include "ctre/phoenix6/CANcoder.hpp"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "lib/subsystems/CanCoderConfig.h"
#include "lib/subsystems/CanCoderIO.h"

class CanCoderIOHardware : public CanCoderIO {
public:
    CanCoderIOHardware(CanCoderConfig config);
    void UpdateFrequency(double hz) override;
    void ReadInputs(CanCoderInputs& inputs) override;

protected:
    ctre::phoenix6::hardware::CANcoder canCoder;
    CanCoderConfig config;

private:
    ctre::phoenix6::StatusSignal<units::turn_t> positionSignal;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocitySignal;
    ctre::phoenix6::BaseStatusSignal* signals[2];

    double goodValues = 0.0;
};
