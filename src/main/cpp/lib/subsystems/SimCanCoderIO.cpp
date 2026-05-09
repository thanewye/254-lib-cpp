#include "lib/subsystems/SimCanCoderIO.h"

#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <units/angle.h>
#include <units/angular_velocity.h>

SimCanCoderIO::SimCanCoderIO(
    const CanCoderConfig &config,
    std::function<SimCanCoderState()> stateSupplier)
    : CanCoderIOHardware(config)
      , m_simState(canCoder.GetSimState())
      , m_stateSupplier(std::move(stateSupplier)) {
    canCoder.SetPosition(0_tr);
}

void SimCanCoderIO::ReadInputs(CanCoderInputs &inputs) {
    const double invertMultiplier =
            config.config.MagnetSensor.SensorDirection ==
            ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive
                ? 1.0
                : -1.0;
    const auto state = m_stateSupplier();
    m_simState.SetRawPosition(
        units::angle::turn_t{invertMultiplier * state.positionRotations});
    m_simState.SetVelocity(
        units::angular_velocity::turns_per_second_t{
            invertMultiplier * state.velocityRotations
        });

    CanCoderIOHardware::ReadInputs(inputs);
}
