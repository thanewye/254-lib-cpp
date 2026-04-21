#pragma once

#include "lib/subsystems/ServoMotorSubsystemConfig.h"
#include "lib/subsystems/CanCoderConfig.h"

struct ServoMotorSubsystemWithCanCoderConfig : public ServoMotorSubsystemConfig {
  CanCoderConfig canCoderConfig;
  double unitsToCancoderRatio = 1.0;
  double canCodergearing = 1.0;
  bool isFusedCancoder = false;

  double GetCanCoderToRotorRatio() const;
};
