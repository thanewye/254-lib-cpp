#pragma once

#include "lib/subsystems/TalonFXIO.h"

// TODO: port from Java SimTalonFXIO — DCMotorSim + TalonFXSimState
class SimTalonFXIO : public TalonFXIO {
 public:
  explicit SimTalonFXIO(const ServoMotorSubsystemConfig& config)
      : TalonFXIO(config) {}

  void UpdateInputs(MotorInputs& inputs) override {}
};
