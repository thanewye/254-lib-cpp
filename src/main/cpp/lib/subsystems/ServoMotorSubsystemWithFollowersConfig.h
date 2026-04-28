#pragma once

#include "lib/subsystems/ServoMotorSubsystemConfig.h"
#include "lib/drivers/CANDeviceId.h"

#include <vector>

struct ServoMotorSubsystemWithFollowersConfig : ServoMotorSubsystemConfig {
  struct FollowerConfig {
    ServoMotorSubsystemConfig config;
    bool inverted = false;
  };

  std::vector<FollowerConfig> followers;
};
