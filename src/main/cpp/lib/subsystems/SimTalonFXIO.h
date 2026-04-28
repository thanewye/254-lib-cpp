#pragma once

#include "lib/subsystems/TalonFXIO.h"

class SimTalonFXIO : public TalonFXIO {
public:
    explicit SimTalonFXIO(const ServoMotorSubsystemConfig& config);

    void ReadInputs(MotorInputs& inputs) override;
};
