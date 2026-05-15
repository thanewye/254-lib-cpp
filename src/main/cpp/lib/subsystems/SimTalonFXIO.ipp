#pragma once

#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/sim/ChassisReference.hpp>
#include <frc/Timer.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "akit/Logger.h"

template<typename pos_t>
frc::DCMotor SimTalonFXIO<pos_t>::GetMotorModel(ServoMotorModel motorModel) {
    switch (motorModel) {
        case ServoMotorModel::KrakenX44:
            return frc::DCMotor::KrakenX44(1);
        case ServoMotorModel::KrakenX60:
        case ServoMotorModel::UNSPECIFIED:
        default:
            return frc::DCMotor::KrakenX60(1);
    }
}

template<typename pos_t>
SimTalonFXIO<pos_t>::SimTalonFXIO(const ServoMotorSubsystemConfig<pos_t>& config)
    : SimTalonFXIO(
        config,
        frc::sim::DCMotorSim(
            frc::LinearSystemId::DCMotorSystem(
                GetMotorModel(config.simMotorModel),
                units::kilogram_square_meter_t{config.momentOfInertia},
                1.0 / config.gearing),
            GetMotorModel(config.simMotorModel),
            {0.001, 0.001})) {}

template<typename pos_t>
SimTalonFXIO<pos_t>::SimTalonFXIO(const ServoMotorSubsystemConfig<pos_t>& config,
                                  frc::sim::DCMotorSim sim)
    : TalonFXIO<pos_t>(config)
      , sim(std::move(sim))
      , kSimVoltageKey(config.name + "/Sim/SimulatorVoltage")
      , kSimPositionRadKey(config.name + "/Sim/SimulatorPositionRadians")
      , kSimRawRotorPositionKey(config.name + "/Sim/setRawRotorPosition")
      , kSimVelocityRadSKey(config.name + "/Sim/SimulatorVelocityRadS")
      , kSetPositionRadKey(config.name + "/Sim/setPositionRad") {
    auto& simState = this->talon.GetSimState();
    simState.Orientation =
            config.fxConfig.MotorOutput.Inverted ==
            ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
                ? ctre::phoenix6::sim::ChassisReference::Clockwise_Positive
                : ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
    if (config.simMotorModel == ServoMotorModel::KrakenX44) {
        simState.SetMotorType(
            ctre::phoenix6::sim::TalonFXSimState::MotorType::KrakenX44);
    }

    simNotifier = std::make_unique<frc::Notifier>(
        [this] { UpdateSimState(); });
    simNotifier->StartPeriodic(5_ms);
}

template<typename pos_t>
void SimTalonFXIO<pos_t>::ReadInputs(MotorInputs& inputs) {
    TalonFXIO<pos_t>::ReadInputs(inputs);
}

template<typename pos_t>
void SimTalonFXIO<pos_t>::SetPositionRad(double radians) {
    const double invertMultiplier =
            this->config.fxConfig.MotorOutput.Inverted ==
            ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
                ? -1.0
                : 1.0;
    sim.SetAngle(units::radian_t{invertMultiplier * radians});
    akit::Logger::RecordOutput(kSetPositionRadKey, radians);
}

template<typename pos_t>
void SimTalonFXIO<pos_t>::OverrideRPS(std::optional<double> rps) {
    overrideRPS = rps;
}

template<typename pos_t>
void SimTalonFXIO<pos_t>::OverridePosition(std::optional<double> radians) {
    overridePositionRad = radians;
}

template<typename pos_t>
void SimTalonFXIO<pos_t>::SetInvertVoltage(bool invertVoltage) {
    this->invertVoltage = invertVoltage;
}

template<typename pos_t>
double SimTalonFXIO<pos_t>::GetIntendedRPS() const {
    return lastRPS.load();
}

template<typename pos_t>
std::function<SimCanCoderIO::SimCanCoderState()>
SimTalonFXIO<pos_t>::GetSupplierForCancoder(
    const ServoMotorSubsystemWithCanCoderConfig<pos_t>& canCoderConfig) {
    return [this, ratio = canCoderConfig.GetCanCoderToRotorRatio()] {
        return SimCanCoderIO::SimCanCoderState{
            .positionRotations = lastRotations.load() * ratio,
            .velocityRotations = lastRPS.load() * ratio,
        };
    };
}

template<typename pos_t>
double SimTalonFXIO<pos_t>::GetSimRatio() const {
    return this->config.gearing;
}

template<typename pos_t>
double SimTalonFXIO<pos_t>::AddFriction(double motorVoltage,
                                        double frictionVoltage) const {
    if (std::abs(motorVoltage) < frictionVoltage) {
        return 0.0;
    }
    if (motorVoltage > 0.0) {
        return motorVoltage - frictionVoltage;
    }
    return motorVoltage + frictionVoltage;
}

template<typename pos_t>
void SimTalonFXIO<pos_t>::UpdateSimState() {
    auto& simState = this->talon.GetSimState();
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
