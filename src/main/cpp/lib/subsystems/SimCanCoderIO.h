#pragma once

#include "lib/subsystems/CanCoderIOHardware.h"

#include <functional>

class SimCanCoderIO : public CanCoderIOHardware {
 public:
  SimCanCoderIO(const CanCoderConfig& config,
                std::function<double()> positionSupplier);

  void UpdateInputs(CanCoderInputs& inputs) override;

 private:
  std::function<double()> m_positionSupplier;
};
