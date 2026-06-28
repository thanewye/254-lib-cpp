#pragma once

#include <vector>

#include "lib/drivers/CANDeviceId.h"
#include "lib/subsystems/ServoMotorSubsystemConfig.h"

template<typename pos_t> struct ServoMotorSubsystemWithFollowersConfig : ServoMotorSubsystemConfig<pos_t> {
    struct FollowerConfig {
        ServoMotorSubsystemConfig<pos_t> config;
        bool inverted = false;
    };

    std::vector<FollowerConfig> followers;
};
