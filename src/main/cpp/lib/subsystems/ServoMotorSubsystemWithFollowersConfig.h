#pragma once

#include <vector>

#include "lib/drivers/CANDeviceId.h"
#include "lib/subsystems/ServoMotorSubsystemConfig.h"

struct ServoMotorSubsystemWithFollowersConfig : ServoMotorSubsystemConfig {
    struct FollowerConfig {
        ServoMotorSubsystemConfig config;
        bool inverted = false;
    };

    std::vector<FollowerConfig> followers;
};
