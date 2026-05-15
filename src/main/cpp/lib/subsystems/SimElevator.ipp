#pragma once

#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/sim/ChassisReference.hpp>
#include <frc/Timer.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "akit/Logger.h"

template<typename pos_t>
SimElevator<pos_t>::SimElevatorTalonFX::SimElevatorTalonFX(
    const ServoMotorSubsystemConfig<pos_t>& config)
    : TalonFXIO<pos_t>(config)
      , simState(this->talon.GetSimState()) {
    simState.Orientation =
            config.fxConfig.MotorOutput.Inverted ==
            ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
                ? ctre::phoenix6::sim::ChassisReference::Clockwise_Positive
                : ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
}

template<typename pos_t>
SimElevator<pos_t>::SimElevator(const ServoMotorSubsystemConfig<pos_t>& config,
                                const SimElevatorConfig& simConfig)
    : config(config)
      , simConfig(simConfig)
      , leadIO(config)
      , sim(
          frc::DCMotor::KrakenX60FOC(1),
          1.0 / simConfig.gearing,
          units::kilogram_t{simConfig.carriageMassKg},
          units::meter_t{simConfig.drumRadiusMeters},
          units::meter_t{config.kMinPosition.value()},
          units::meter_t{config.kMaxPosition.value()},
          simConfig.simulateGravity,
          units::meter_t{simConfig.startingHeightMeters},
          {0.0, 0.0}) {
    StartNotifier();
}

template<typename pos_t>
SimElevator<pos_t>::SimElevator(
    const ServoMotorSubsystemWithFollowersConfig<pos_t>& config,
    const SimElevatorConfig& simConfig)
    : config(static_cast<const ServoMotorSubsystemConfig<pos_t>&>(config))
      , simConfig(simConfig)
      , leadIO(config)
      , sim(
          frc::DCMotor::KrakenX60FOC(1 + static_cast<int>(config.followers.size())),
          1.0 / simConfig.gearing,
          units::kilogram_t{simConfig.carriageMassKg},
          units::meter_t{simConfig.drumRadiusMeters},
          units::meter_t{config.kMinPosition.value()},
          units::meter_t{config.kMaxPosition.value()},
          simConfig.simulateGravity,
          units::meter_t{simConfig.startingHeightMeters},
          {0.0, 0.0}) {
    followerInverted.reserve(config.followers.size());
    for (const auto& follower : config.followers) {
        followerIOs.push_back(
            std::make_unique<SimElevatorTalonFX>(follower.config));
        followerInverted.push_back(follower.inverted);
    }
    StartNotifier();
}

template<typename pos_t>
TalonFXIO<pos_t>* SimElevator<pos_t>::GetLeadIO() {
    return &leadIO;
}

template<typename pos_t>
std::vector<TalonFXIO<pos_t>*> SimElevator<pos_t>::GetFollowerIO() const {
    std::vector<TalonFXIO<pos_t>*> followerIO;
    followerIO.reserve(followerIOs.size());
    for (auto& follower : followerIOs) {
        followerIO.push_back(follower.get());
    }
    return followerIO;
}

template<typename pos_t>
void SimElevator<pos_t>::StartNotifier() {
    simNotifier = std::make_unique<frc::Notifier>(
        [this] { UpdateSimState(); });
    simNotifier->StartPeriodic(5_ms);
}

template<typename pos_t>
double SimElevator<pos_t>::AddFriction(double motorVoltage,
                                       double frictionVoltage) {
    if (std::abs(motorVoltage) < frictionVoltage) {
        return 0.0;
    }
    if (motorVoltage > 0.0) {
        return motorVoltage - frictionVoltage;
    }
    return motorVoltage + frictionVoltage;
}

template<typename pos_t>
void SimElevator<pos_t>::UpdateSimState() {
    auto& simState = leadIO.simState;
    const double rawVoltage = simState.GetMotorVoltage().value();
    const double simVoltage = AddFriction(rawVoltage, 0.25);

    sim.SetInputVoltage(units::volt_t{simVoltage});
    akit::Logger::RecordOutput(config.name + "/Sim/SimulatorVoltage",
                               simVoltage);
    akit::Logger::RecordOutput(config.name + "/Sim/RawMotorVoltage",
                               rawVoltage);

    const auto timestamp = frc::Timer::GetFPGATimestamp();
    sim.Update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    const double simPositionMeters = sim.GetPosition().value();
    akit::Logger::RecordOutput(config.name + "/Sim/SimulatorPositionMeters",
                               simPositionMeters);

    const double rotorPosition =
            simPositionMeters / simConfig.meterToRotorRatio;
    simState.SetRawRotorPosition(units::angle::turn_t{rotorPosition});
    akit::Logger::RecordOutput(config.name + "/Sim/setRawRotorPosition",
                               rotorPosition);

    const double rotorVelocity =
            sim.GetVelocity().value() / simConfig.meterToRotorRatio;
    simState.SetRotorVelocity(
        units::angular_velocity::turns_per_second_t{rotorVelocity});
    akit::Logger::RecordOutput(config.name + "/Sim/SimulatorVelocityMS",
                               sim.GetVelocity().value());

    for (size_t i = 0; i < followerIOs.size(); ++i) {
        const double multiplier = followerInverted[i] ? 1.0 : -1.0;
        followerIOs[i]->simState.SetRawRotorPosition(
            units::angle::turn_t{rotorPosition * multiplier});
        followerIOs[i]->simState.SetRotorVelocity(
            units::angular_velocity::turns_per_second_t{
                rotorVelocity * multiplier
            });
    }
}
