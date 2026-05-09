#pragma once

#include <functional>

#include <ctre/phoenix6/sim/CANcoderSimState.hpp>

#include "lib/subsystems/CanCoderIOHardware.h"

class SimCanCoderIO : public CanCoderIOHardware {
public:
    struct SimCanCoderState {
        double positionRotations = std::numeric_limits<double>::quiet_NaN();
        double velocityRotations = std::numeric_limits<double>::quiet_NaN();
    };

    SimCanCoderIO(
        const CanCoderConfig &config,
        std::function<SimCanCoderState()> stateSupplier);

    void ReadInputs(CanCoderInputs &inputs) override;

private:
    ctre::phoenix6::sim::CANcoderSimState &m_simState;
    std::function<SimCanCoderState()> m_stateSupplier;
};
