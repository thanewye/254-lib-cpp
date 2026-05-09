#include "lib/subsystems/SimTalonFXIO.h"

#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/sim/ChassisReference.hpp>
#include <frc/Timer.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "akit/Logger.h"

frc::DCMotor SimTalonFXIO::GetMotorModel(
    ServoMotorSubsystemConfig::Motor motorModel) {
    switch (motorModel) {
        case ServoMotorSubsystemConfig::Motor::KrakenX44:
            return frc::DCMotor::KrakenX44(1);
        case ServoMotorSubsystemConfig::Motor::KrakenX60:
        case ServoMotorSubsystemConfig::Motor::UNSPECIFIED:
        default:
            return frc::DCMotor::KrakenX60(1);
    }
}

SimTalonFXIO::SimTalonFXIO(const ServoMotorSubsystemConfig &config)
    : SimTalonFXIO(
        config,
        frc::sim::DCMotorSim(
            frc::LinearSystemId::DCMotorSystem(
                GetMotorModel(config.simMotorModel),
                units::kilogram_square_meter_t{config.momentOfInertia},
                1.0 / config.gearing),
            GetMotorModel(config.simMotorModel),
            {0.001, 0.001})) {}

SimTalonFXIO::SimTalonFXIO(const ServoMotorSubsystemConfig &config,
                           frc::sim::DCMotorSim sim)
    : TalonFXIO(config)
      , sim(std::move(sim))
      , kSimVoltageKey(config.name + "/Sim/SimulatorVoltage")
      , kSimPositionRadKey(config.name + "/Sim/SimulatorPositionRadians")
      , kSimRawRotorPositionKey(config.name + "/Sim/setRawRotorPosition")
      , kSimVelocityRadSKey(config.name + "/Sim/SimulatorVelocityRadS")
      , kSetPositionRadKey(config.name + "/Sim/setPositionRad") {
    auto &simState = talon.GetSimState();
    simState.Orientation =
            config.fxConfig.MotorOutput.Inverted ==
            ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
                ? ctre::phoenix6::sim::ChassisReference::Clockwise_Positive
                : ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    if (config.simMotorModel == ServoMotorSubsystemConfig::Motor::KrakenX44) {
        simState.SetMotorType(
            ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX44);
    }

    simNotifier = std::make_unique<frc::Notifier>(
        [this] { UpdateSimState(); });
    simNotifier->StartPeriodic(5_ms);
}

void SimTalonFXIO::ReadInputs(MotorInputs &inputs) {
    TalonFXIO::ReadInputs(inputs);
}

void SimTalonFXIO::SetPositionRad(double radians) {
    const double invertMultiplier =
            config.fxConfig.MotorOutput.Inverted ==
            ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
                ? -1.0
                : 1.0;
    sim.SetAngle(units::radian_t{invertMultiplier * radians});
    akit::Logger::RecordOutput(kSetPositionRadKey, radians);
}

void SimTalonFXIO::OverrideRPS(std::optional<double> rps) {
    overrideRPS = rps;
}

void SimTalonFXIO::OverridePosition(std::optional<double> radians) {
    overridePositionRad = radians;
}

void SimTalonFXIO::SetInvertVoltage(bool invertVoltage) {
    this->invertVoltage = invertVoltage;
}

double SimTalonFXIO::GetIntendedRPS() const {
    return lastRPS.load();
}

std::function<SimCanCoderIO::SimCanCoderState()>
SimTalonFXIO::GetSupplierForCancoder(
    const ServoMotorSubsystemWithCanCoderConfig &canCoderConfig) {
    return [this, ratio = canCoderConfig.GetCanCoderToRotorRatio()] {
        return SimCanCoderIO::SimCanCoderState{
            .positionRotations = lastRotations.load() * ratio,
            .velocityRotations = lastRPS.load() * ratio,
        };
    };
}

double SimTalonFXIO::GetSimRatio() const {
    return config.gearing;
}

double SimTalonFXIO::AddFriction(double motorVoltage,
                                 double frictionVoltage) const {
    if (std::abs(motorVoltage) < frictionVoltage) {
        return 0.0;
    }
    if (motorVoltage > 0.0) {
        return motorVoltage - frictionVoltage;
    }
    return motorVoltage + frictionVoltage;
}

void SimTalonFXIO::UpdateSimState() {
    auto &simState = talon.GetSimState();
    double simVoltage =
            AddFriction(simState.GetMotorVoltage().value(), 0.1);
    if (invertVoltage) {
        simVoltage = -simVoltage;
    }
    sim.SetInputVoltage(units::volt_t{simVoltage});
    akit::Logger::RecordOutput(kSimVoltageKey, simVoltage);

    const auto timestamp = frc::Timer::GetFPGATimestamp();
    sim.Update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    if (overridePositionRad.has_value()) {
        sim.SetAngle(units::radian_t{*overridePositionRad});
    }

    const double simPositionRads = sim.GetAngularPosition().value();
    akit::Logger::RecordOutput(kSimPositionRadKey, simPositionRads);

    const double rotorPosition =
            units::angle::turn_t{sim.GetAngularPosition()}.value() / GetSimRatio();
    lastRotations.store(rotorPosition);
    simState.SetRawRotorPosition(units::angle::turn_t{rotorPosition});
    akit::Logger::RecordOutput(kSimRawRotorPositionKey, rotorPosition);

    const double rotorVelocity =
            units::angular_velocity::turns_per_second_t{
                sim.GetAngularVelocity()
            }
            .value() /
            GetSimRatio();
    lastRPS.store(rotorVelocity);
    simState.SetRotorVelocity(units::angular_velocity::turns_per_second_t{
        overrideRPS.value_or(rotorVelocity)
    });
    akit::Logger::RecordOutput(kSimVelocityRadSKey, sim.GetAngularVelocity().value());
}
