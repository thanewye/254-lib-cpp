#pragma once

#include "lib/subsystems/CanCoderIO.h"
#include "lib/subsystems/CanCoderConfig.h"

#include <ctre/phoenix6/CANcoder.hpp>

class CanCoderIOHardware : public CanCoderIO {
 public:
  explicit CanCoderIOHardware(const CanCoderConfig& config);

  void UpdateInputs(CanCoderInputs& inputs) override;
  void UpdateFrequency(double hz) override;

 protected:
  ctre::phoenix6::hardware::CANcoder m_cancoder;
};
