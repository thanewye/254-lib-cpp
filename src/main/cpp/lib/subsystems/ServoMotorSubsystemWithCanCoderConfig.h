#pragma once

#include "lib/subsystems/CanCoderConfig.h"
#include "lib/subsystems/ServoMotorSubsystemConfig.h"

struct ServoMotorSubsystemWithCanCoderConfig : ServoMotorSubsystemConfig {
    CanCoderConfig canCoderConfig;
    double unitsToCancoderRatio = 1.0;
    double canCodergearing = 1.0;
    bool isFusedCancoder = false;

    double GetCanCoderToRotorRatio() const {
        return gearing / canCodergearing;
    }
};
