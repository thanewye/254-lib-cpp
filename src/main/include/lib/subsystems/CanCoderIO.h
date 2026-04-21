#pragma once

#include "lib/subsystems/CanCoderInputs.h"

class CanCoderIO {
 public:
  virtual ~CanCoderIO() = default;
  virtual void UpdateInputs(CanCoderInputs& inputs) = 0;
  virtual void UpdateFrequency(double hz) = 0;
};
