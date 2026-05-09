#include "lib/subsystems/SimElevator.h"

#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/sim/ChassisReference.hpp>
#include <frc/Timer.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "akit/Logger.h"

SimElevator::SimElevatorTalonFX::SimElevatorTalonFX(
    const ServoMotorSubsystemConfig &config)
    : TalonFXIO(config)
      , simState(talon.GetSimState()) {
    simState.Orientation =
            config.fxConfig.MotorOutput.Inverted ==
            ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
                ? ctre::phoenix6::sim::ChassisReference::Clockwise_Positive
                : ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
}

SimElevator::SimElevator(const ServoMotorSubsystemConfig &config,
                         const SimElevatorConfig &simConfig)
    : config(config)
      , simConfig(simConfig)
      , leadIO(config)
      , sim(
          frc::DCMotor::KrakenX60FOC(1),
          1.0 / simConfig.gearing,
          units::kilogram_t{simConfig.carriageMassKg},
          units::meter_t{simConfig.drumRadiusMeters},
          units::meter_t{config.kMinPositionUnits},
          units::meter_t{config.kMaxPositionUnits},
          simConfig.simulateGravity,
          units::meter_t{simConfig.startingHeightMeters},
          {0.0, 0.0}) {
    StartNotifier();
}

SimElevator::SimElevator(
    const ServoMotorSubsystemWithFollowersConfig &config,
    const SimElevatorConfig &simConfig)
    : config(static_cast<const ServoMotorSubsystemConfig &>(config))
      , simConfig(simConfig)
      , leadIO(config)
      , sim(
          frc::DCMotor::KrakenX60FOC(1 + static_cast<int>(config.followers.size())),
          1.0 / simConfig.gearing,
          units::kilogram_t{simConfig.carriageMassKg},
          units::meter_t{simConfig.drumRadiusMeters},
          units::meter_t{config.kMinPositionUnits},
          units::meter_t{config.kMaxPositionUnits},
          simConfig.simulateGravity,
          units::meter_t{simConfig.startingHeightMeters},
          {0.0, 0.0}) {
    followerInverted.reserve(config.followers.size());
    for (const auto &follower: config.followers) {
        followerIOs.push_back(
            std::make_unique<SimElevatorTalonFX>(follower.config));
        followerInverted.push_back(follower.inverted);
    }
    StartNotifier();
}

TalonFXIO *SimElevator::GetLeadIO() {
    return &leadIO;
}

std::vector<TalonFXIO *> SimElevator::GetFollowerIO() const {
    std::vector<TalonFXIO *> followerIO;
    followerIO.reserve(followerIOs.size());
    for (auto &follower: followerIOs) {
        followerIO.push_back(follower.get());
    }
    return followerIO;
}

void SimElevator::StartNotifier() {
    simNotifier = std::make_unique<frc::Notifier>(
        [this] { UpdateSimState(); });
    simNotifier->StartPeriodic(5_ms);
}

double SimElevator::AddFriction(double motorVoltage,
                                double frictionVoltage) {
    if (std::abs(motorVoltage) < frictionVoltage) {
        return 0.0;
    }
    if (motorVoltage > 0.0) {
        return motorVoltage - frictionVoltage;
    }
    return motorVoltage + frictionVoltage;
}

void SimElevator::UpdateSimState() {
    auto &simState = leadIO.simState;
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
