#pragma once

#include <functional>

#include "lib/subsystems/CanCoderIOHardware.h"

class SimCanCoderIO : public CanCoderIOHardware {
 public:
  SimCanCoderIO(const CanCoderConfig& config,
                std::function<double()> positionSupplier);

  void ReadInputs(CanCoderInputs& inputs) override;

 private:
  std::function<double()> m_positionSupplier;
};
