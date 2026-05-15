#pragma once

#include <memory>
#include <utility>

#include "akit/Logger.h"
#include "frc/Timer.h"
#include "frc2/command/Commands.h"
#include "lib/util/Util.h"

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
ServoMotorSubsystem<pos_t, T, U>::ServoMotorSubsystem(
    const ServoMotorSubsystemConfig<pos_t>& config, T inputs, U* io)
    : frc2::SubsystemBase(config.name)
      , io(io)
      , inputs(std::move(inputs))
      , conf(config)
      , defaultSupplyCurrentLimit(config.fxConfig.CurrentLimits.SupplyCurrentLimit) {
    SetDefaultCommand(DutyCycleCommand([] { return 0.0; })
        .WithName(LogKey(" Default Command Neutral"))
        .IgnoringDisable(true));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::Periodic() {
    double timestamp = static_cast<double>(frc::Timer::GetFPGATimestamp());
    io->ReadInputs(inputs);

    const bool globalLoadShedding = conf.loadSheddingEnabled && conf.loadSheddingCondition();
    if (globalLoadShedding != isLoadSheddingActive) {
        isLoadSheddingActive = globalLoadShedding;
        SetSupplyCurrentLimitImpl(
            isLoadSheddingActive
                ? conf.loadSheddingSupplyCurrentLimitAmps
                : defaultSupplyCurrentLimit);
    }

    akit::Logger::ProcessInputs(GetName(), inputs);
    akit::Logger::RecordOutput(
        LogKey("/latencyPeriodicSec"),
        static_cast<double>(frc::Timer::GetFPGATimestamp()) - timestamp);
    akit::Logger::RecordOutput(
        LogKey("/currentCommand"),
        GetCurrentCommand() == nullptr ? "Default" : GetCurrentCommand()->GetName());
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetCurrentPosition(pos_t position) {
    io->SetCurrentPosition(position.value());
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetCurrentPositionAsZero() {
    io->SetCurrentPositionAsZero();
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
std::string ServoMotorSubsystem<pos_t, T, U>::LogKey(std::string_view suffix) {
    auto [it, inserted] = logKeyCache.try_emplace(
        std::string{suffix}, GetName() + std::string{suffix});
    return it->second;
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetOpenLoopDutyCycleImpl(double dutyCycle) {
    akit::Logger::RecordOutput(LogKey("/API/setOpenLoopDutyCycle/dutyCycle"), dutyCycle);
    io->SetOpenLoopDutyCycle(dutyCycle);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetOpenLoopDutyCycleNoFOCImpl(double dutyCycle) {
    akit::Logger::RecordOutput(LogKey("/API/setOpenLoopDutyCycleNoFOC/dutyCycle"), dutyCycle);
    io->SetOpenLoopDutyCycleNoFOC(dutyCycle);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetOpenLoopDutyCycleIgnoreLimitsImpl(double dutyCycle) {
    akit::Logger::RecordOutput(LogKey("/API/setOpenLoopDutyCycleIgnoreLimits/dutyCycle"), dutyCycle);
    io->SetOpenLoopDutyCycleIgnoreLimits(dutyCycle);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVoltageImpl(double voltage) {
    akit::Logger::RecordOutput(LogKey("/API/setVoltageImpl/voltage"), voltage);
    io->SetVoltageOutput(voltage);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue mode) {
    std::string modeName = "Unknown";
    if (mode == ctre::phoenix6::signals::NeutralModeValue::Coast) {
        modeName = "Coast";
    } else if (mode == ctre::phoenix6::signals::NeutralModeValue::Brake) {
        modeName = "Brake";
    }
    akit::Logger::RecordOutput(LogKey("/API/setNeutralModeImpl/Mode"), modeName);
    io->SetNeutralMode(mode);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetPositionSetpointImpl(double units) {
    positionSetpoint = pos_t{units};
    akit::Logger::RecordOutput(LogKey("/API/setPositionSetpointImp/Units"), units);
    io->SetPositionSetpoint(units);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetPositionTorqueCurrentFOCImpl(double units) {
    positionSetpoint = pos_t{units};
    akit::Logger::RecordOutput(LogKey("/API/setPositionTorqueCurrentFOCImp/Units"), units);
    io->SetPositionTorqueCurrentFOC(units);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetMotionMagicSetpointImpl(double units, int slot) {
    positionSetpoint = pos_t{units};
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImp/Units"), units);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImp/Slot"), slot);
    io->SetMotionMagicSetpoint(units, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetMotionMagicSetpointImpl(
    double units, const ctre::phoenix6::configs::MotionMagicConfigs& config, int slot) {
    positionSetpoint = pos_t{units};
    double velocity = config.MotionMagicCruiseVelocity.value();
    double acceleration = config.MotionMagicAcceleration.value();
    double jerk = config.MotionMagicJerk.value();

    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImpDynamic/Units"), units);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImpDynamic/Velocity"), velocity);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImpDynamic/Accel"), acceleration);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImpDynamic/Jerk"), jerk);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImpDynamic/Slot"), slot);
    io->SetMotionMagicSetpoint(units, velocity, acceleration, jerk, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetMotionMagicSetpointImpl(
    double units, const ctre::phoenix6::configs::MotionMagicConfigs& config, double feedforward, int slot) {
    positionSetpoint = pos_t{units};
    double velocity = config.MotionMagicCruiseVelocity.value();
    double acceleration = config.MotionMagicAcceleration.value();
    double jerk = config.MotionMagicJerk.value();

    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImpDynamic/Units"), units);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImpDynamic/Velocity"), velocity);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImpDynamic/Accel"), acceleration);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImpDynamic/Jerk"), jerk);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImpDynamic/Slot"), slot);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImpDynamic/Feedforwards"), feedforward);
    io->SetMotionMagicSetpoint(units, velocity, acceleration, jerk, slot, feedforward);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetMotionMagicTorqueCurrentFOCImpl(double units, int slot) {
    positionSetpoint = pos_t{units};
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImp/Units"), units);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImp/Slot"), slot);
    io->SetMotionMagicTorqueCurrentFOC(units, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetMotionMagicTorqueCurrentFOCImpl(
    double units, const ctre::phoenix6::configs::MotionMagicConfigs& config, int slot) {
    positionSetpoint = pos_t{units};
    double velocity = config.MotionMagicCruiseVelocity.value();
    double acceleration = config.MotionMagicAcceleration.value();
    double jerk = config.MotionMagicJerk.value();

    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImpDynamic/Units"), units);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImpDynamic/Velocity"), velocity);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImpDynamic/Accel"), acceleration);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImpDynamic/Jerk"), jerk);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImpDynamic/Slot"), slot);
    io->SetMotionMagicTorqueCurrentFOC(units, velocity, acceleration, jerk, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetMotionMagicTorqueCurrentFOCImpl(
    double units, const ctre::phoenix6::configs::MotionMagicConfigs& config, double feedforward, int slot) {
    positionSetpoint = pos_t{units};
    double velocity = config.MotionMagicCruiseVelocity.value();
    double acceleration = config.MotionMagicAcceleration.value();
    double jerk = config.MotionMagicJerk.value();

    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImpDynamic/Units"), units);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImpDynamic/Velocity"), velocity);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImpDynamic/Accel"), acceleration);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImpDynamic/Jerk"), jerk);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImpDynamic/Slot"), slot);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImpDynamic/Feedforwards"), feedforward);
    io->SetMotionMagicTorqueCurrentFOC(units, velocity, acceleration, jerk, slot, feedforward);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVelocitySetpointImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocitySetpointImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocitySetpoint(unitsPerSecond, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVelocitySetpointNoFOCImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocitySetpointNoFOCImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocitySetpointNoFOC(unitsPerSecond, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVelocitySetpointIgnoreLimitsImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocitySetpointIgnoreLimitsImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocitySetpointIgnoreLimits(unitsPerSecond, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVelocitySetpointNoFOCIgnoreLimitsImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocitySetpointNoFOCIgnoreLimitsImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocitySetpointNoFOCIgnoreLimits(unitsPerSecond, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVelocityMotionMagicSetpointImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicSetpointImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocityMotionMagicSetpoint(unitsPerSecond, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVelocityMotionMagicSetpointImpl(
    double unitsPerSecond, int slot, double feedforward) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicSetpointImpl/UnitsPerS"), unitsPerSecond);
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicSetpointImpl/UnitsPerS/feedforward"),
                               feedforward);
    io->SetVelocityMotionMagicSetpoint(unitsPerSecond, slot, feedforward);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVelocityMotionMagicSetpointNoFOCImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicSetpointNoFOCImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocityMotionMagicSetpointNoFOC(unitsPerSecond, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVelocityMotionMagicTorqueCurrentFOCImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicTorqueCurrentFOCImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocityMotionMagicTorqueCurrentFOC(unitsPerSecond, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVelocityMotionMagicSetpointIgnoreLimitsImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicSetpointIgnoreLimitsImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocityMotionMagicSetpointIgnoreLimits(unitsPerSecond, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVelocityMotionMagicSetpointNoFOCIgnoreLimitsImpl(
    double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicSetpointNoFOCIgnoreLimitsImpl/UnitsPerS"),
                               unitsPerSecond);
    io->SetVelocityMotionMagicSetpointNoFOCIgnoreLimits(unitsPerSecond, slot);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetTorqueCurrentFOCImpl(double current) {
    akit::Logger::RecordOutput(LogKey("/API/setTorqueCurrentFoC/Current"), current);
    io->SetTorqueCurrentFOC(current);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetVelocityTorqueCurrentFOCImpl(double velocity, double feedforward) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityTorqueCurrentFOC/Velocity"), velocity);
    akit::Logger::RecordOutput(LogKey("/API/setVelocityTorqueCurrentFOC/Feedforward"), feedforward);
    io->SetVelocityTorqueCurrentFOC(velocity, feedforward);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetSupplyCurrentLimitImpl(double amps) {
    akit::Logger::RecordOutput(LogKey("/API/setSupplyCurrentLimit/Amps"), amps);
    io->SetSupplyCurrentLimit(amps);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetStatorCurrentLimitImpl(double amps) {
    akit::Logger::RecordOutput(LogKey("/API/setStatorCurrentLimit/Amps"), amps);
    io->SetStatorCurrentLimit(amps);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetMotionMagicConfig(
    const ctre::phoenix6::configs::MotionMagicConfigs& config) {
    io->SetMotionMagicConfig(config);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::WithoutLimitsTemporarily() {
    struct PreviousLimits {
        bool forward = false;
        bool reverse = false;
    };
    auto previous = std::make_shared<PreviousLimits>();

    return frc2::cmd::StartEnd(
        [this, previous] {
            previous->forward = conf.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable;
            previous->reverse = conf.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable;
            io->SetEnableSoftLimits(false, false);
        },
        [this, previous] {
            io->SetEnableSoftLimits(previous->forward, previous->reverse);
        });
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::DutyCycleCommand(std::function<double()> dutyCycleSupplier) {
    return RunEnd(
                [this, dutyCycleSupplier] { SetOpenLoopDutyCycleImpl(dutyCycleSupplier()); },
                [this] { SetOpenLoopDutyCycleImpl(0.0); })
            .WithName(LogKey(" DutyCycleControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::DutyCycleCommandNoEnd(std::function<double()> dutyCycleSupplier) {
    return RunEnd(
                [this, dutyCycleSupplier] { SetOpenLoopDutyCycleImpl(dutyCycleSupplier()); },
                [] {})
            .WithName(LogKey(" DutyCycleControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::DutyCycleIgnoreLimitsCommand(
    std::function<double()> dutyCycleSupplier) {
    return RunEnd(
                [this, dutyCycleSupplier] { SetOpenLoopDutyCycleIgnoreLimitsImpl(dutyCycleSupplier()); },
                [this] { SetOpenLoopDutyCycleIgnoreLimitsImpl(0.0); })
            .WithName(LogKey(" DutyCycleControlIgnoreLimits"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::DutyCycleNoFOCCommand(std::function<double()> dutyCycleSupplier) {
    return RunEnd(
                [this, dutyCycleSupplier] { SetOpenLoopDutyCycleNoFOCImpl(dutyCycleSupplier()); },
                [this] { SetOpenLoopDutyCycleNoFOCImpl(0.0); })
            .WithName(LogKey(" DutyCycleNoFOCControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T,
    U>::DutyCycleNoFOCCommandNoEnd(std::function<double()> dutyCycleSupplier) {
    return RunEnd(
                [this, dutyCycleSupplier] { SetOpenLoopDutyCycleNoFOCImpl(dutyCycleSupplier()); },
                [] {})
            .WithName(LogKey(" DutyCycleNoFOCControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VoltageCommand(std::function<double()> voltageSupplier) {
    return RunEnd(
                [this, voltageSupplier] { SetVoltageImpl(voltageSupplier()); },
                [this] { SetVoltageImpl(0.0); })
            .WithName(LogKey(" VoltageControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::SetTorqueCurrentFOCCommand(std::function<double()> currentSupplier) {
    return RunEnd(
                [this, currentSupplier] { SetTorqueCurrentFOCImpl(currentSupplier()); },
                [] {})
            .WithName(LogKey(" torqueCurrentFOCCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T,
    U>::VelocityTorqueCurrentCommand(std::function<vel_t()> velocitySupplier) {
    return RunEnd(
                [this, velocitySupplier] { SetVelocityTorqueCurrentFOCImpl(velocitySupplier().value(), 0.0); },
                [] {})
            .WithName(LogKey(" velocityTorqueCurrentCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityTorqueCurrentCommand(
    std::function<vel_t()> velocitySupplier, std::function<double()> feedforwardSupplier) {
    return RunEnd(
                [this, velocitySupplier, feedforwardSupplier] {
                    SetVelocityTorqueCurrentFOCImpl(velocitySupplier().value(), feedforwardSupplier());
                },
                [] {})
            .WithName(LogKey(" velocityTorqueCurrentCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::PositionSetpointCommand(std::function<pos_t()> unitSupplier) {
    return RunEnd(
                [this, unitSupplier] { SetPositionSetpointImpl(unitSupplier().value()); },
                [] {})
            .WithName(LogKey(" positionSetpointCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::PositionSetpointUntilOnTargetCommand(
    std::function<pos_t()> unitSupplier, std::function<pos_t()> epsilonSupplier) {
    return PositionSetpointCommand(unitSupplier)
            .Until([this, unitSupplier, epsilonSupplier] {
                return Util::EpsilonEquals(unitSupplier().value(), inputs.unitPosition, epsilonSupplier().value());
            });
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T,
    U>::PositionTorqueCurrentFOCCommand(std::function<pos_t()> unitSupplier) {
    return RunEnd(
                [this, unitSupplier] { SetPositionTorqueCurrentFOCImpl(unitSupplier().value()); },
                [] {})
            .WithName(LogKey(" positionTorqueCurrentFOCCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicSetpointCommand(std::function<pos_t()> unitSupplier) {
    return MotionMagicSetpointCommand(unitSupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicSetpointCommand(
    std::function<pos_t()> unitSupplier, int slot) {
    return RunEnd(
                [this, unitSupplier, slot] { SetMotionMagicSetpointImpl(unitSupplier().value(), slot); },
                [] {})
            .WithName(LogKey(" motionMagicSetpointCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicSetpointCommand(
    std::function<pos_t()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier) {
    return MotionMagicSetpointCommand(unitSupplier, configSupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicSetpointCommand(
    std::function<pos_t()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    int slot) {
    return RunEnd(
                [this, unitSupplier, configSupplier, slot] {
                    SetMotionMagicSetpointImpl(unitSupplier().value(), configSupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" dynamicMotionMagicSetpointCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicSetpointCommand(
    std::function<pos_t()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    std::function<double()> feedforwardSupplier,
    int slot) {
    return RunEnd(
                [this, unitSupplier, configSupplier, feedforwardSupplier, slot] {
                    SetMotionMagicSetpointImpl(unitSupplier().value(), configSupplier(), feedforwardSupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" dynamicMotionMagicSetpointCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicSetpointCommandBlocking(
    std::function<pos_t()> setpointSupplier, pos_t tolerance) {
    return MotionMagicSetpointCommandBlocking(setpointSupplier, tolerance, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicSetpointCommandBlocking(
    std::function<pos_t()> setpointSupplier, pos_t tolerance, int slot) {
    return MotionMagicSetpointCommand([setpointSupplier] { return setpointSupplier(); }, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(
                    GetCurrentPosition().value(), setpointSupplier().value(), tolerance.value());
            });
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicSetpointCommandBlocking(
    std::function<pos_t()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    pos_t tolerance) {
    return MotionMagicSetpointCommandBlocking(setpointSupplier, configSupplier, tolerance, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicSetpointCommandBlocking(
    std::function<pos_t()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    pos_t tolerance,
    int slot) {
    return MotionMagicSetpointCommand(setpointSupplier, configSupplier, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(
                    GetCurrentPosition().value(), setpointSupplier().value(), tolerance.value());
            });
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicSetpointCommandBlocking(
    std::function<pos_t()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    std::function<double()> feedforwardSupplier,
    pos_t tolerance,
    int slot) {
    return MotionMagicSetpointCommand(setpointSupplier, configSupplier, feedforwardSupplier, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(
                    GetCurrentPosition().value(), setpointSupplier().value(), tolerance.value());
            });
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicTorqueCurrentFOCCommand(
    std::function<pos_t()> unitSupplier) {
    return MotionMagicTorqueCurrentFOCCommand(unitSupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicTorqueCurrentFOCCommand(
    std::function<pos_t()> unitSupplier, int slot) {
    return RunEnd(
                [this, unitSupplier, slot] { SetMotionMagicTorqueCurrentFOCImpl(unitSupplier().value(), slot); },
                [] {})
            .WithName(LogKey(" motionMagicTorqueCurrentFOCCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicTorqueCurrentFOCCommand(
    std::function<pos_t()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier) {
    return MotionMagicTorqueCurrentFOCCommand(unitSupplier, configSupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicTorqueCurrentFOCCommand(
    std::function<pos_t()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    int slot) {
    return RunEnd(
                [this, unitSupplier, configSupplier, slot] {
                    SetMotionMagicTorqueCurrentFOCImpl(unitSupplier().value(), configSupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" dynamicMotionMagicTorqueCurrentFOCCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicTorqueCurrentFOCCommand(
    std::function<pos_t()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    std::function<double()> feedforwardSupplier,
    int slot) {
    return RunEnd(
                [this, unitSupplier, configSupplier, feedforwardSupplier, slot] {
                    SetMotionMagicTorqueCurrentFOCImpl(
                        unitSupplier().value(), configSupplier(), feedforwardSupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" dynamicMotionMagicTorqueCurrentFOCCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicTorqueCurrentFOCCommandBlocking(
    std::function<pos_t()> setpointSupplier, pos_t tolerance) {
    return MotionMagicTorqueCurrentFOCCommandBlocking(setpointSupplier, tolerance, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicTorqueCurrentFOCCommandBlocking(
    std::function<pos_t()> setpointSupplier, pos_t tolerance, int slot) {
    return MotionMagicTorqueCurrentFOCCommand([setpointSupplier] { return setpointSupplier(); }, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(
                    GetCurrentPosition().value(), setpointSupplier().value(), tolerance.value());
            });
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicTorqueCurrentFOCCommandBlocking(
    std::function<pos_t()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    pos_t tolerance) {
    return MotionMagicTorqueCurrentFOCCommandBlocking(setpointSupplier, configSupplier, tolerance, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicTorqueCurrentFOCCommandBlocking(
    std::function<pos_t()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    pos_t tolerance,
    int slot) {
    return MotionMagicTorqueCurrentFOCCommand(setpointSupplier, configSupplier, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(
                    GetCurrentPosition().value(), setpointSupplier().value(), tolerance.value());
            });
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::MotionMagicTorqueCurrentFOCCommandBlocking(
    std::function<pos_t()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    std::function<double()> feedforwardSupplier,
    pos_t tolerance,
    int slot) {
    return MotionMagicTorqueCurrentFOCCommand(setpointSupplier, configSupplier, feedforwardSupplier, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(
                    GetCurrentPosition().value(), setpointSupplier().value(), tolerance.value());
            });
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocitySetpointCommand(
    std::function<vel_t()> velocitySupplier) {
    return VelocitySetpointCommand(velocitySupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocitySetpointCommand(
    std::function<vel_t()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] { SetVelocitySetpointImpl(velocitySupplier().value(), slot); },
                [] {})
            .WithName(LogKey(" VelocityControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocitySetpointNoFOCCommand(
    std::function<vel_t()> velocitySupplier) {
    return VelocitySetpointNoFOCCommand(velocitySupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocitySetpointNoFOCCommand(
    std::function<vel_t()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] { SetVelocitySetpointNoFOCImpl(velocitySupplier().value(), slot); },
                [] {})
            .WithName(LogKey(" VelocityNoFOCControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointCommand(
    std::function<vel_t()> velocitySupplier) {
    return VelocityMotionMagicSetpointCommand(velocitySupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointCommand(
    std::function<vel_t()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocityMotionMagicSetpointImpl(velocitySupplier().value(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointCommand(
    std::function<vel_t()> velocitySupplier,
    std::function<double()> feedforwardSupplier,
    int slot) {
    return RunEnd(
                [this, velocitySupplier, feedforwardSupplier, slot] {
                    SetVelocityMotionMagicSetpointImpl(
                        velocitySupplier().value(), slot, feedforwardSupplier());
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointNoEndCommand(
    std::function<vel_t()> velocitySupplier) {
    return VelocityMotionMagicSetpointNoEndCommand(velocitySupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointNoEndCommand(
    std::function<vel_t()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocityMotionMagicSetpointImpl(velocitySupplier().value(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointNoEndCommand(
    std::function<vel_t()> velocitySupplier, int slot, std::function<double()> feedforwardSupplier) {
    return RunEnd(
                [this, velocitySupplier, feedforwardSupplier, slot] {
                    SetVelocityMotionMagicSetpointImpl(
                        velocitySupplier().value(), slot, feedforwardSupplier());
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointNoFOCNoEndCommand(
    std::function<vel_t()> velocitySupplier) {
    return VelocityMotionMagicSetpointNoFOCNoEndCommand(velocitySupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointNoFOCNoEndCommand(
    std::function<vel_t()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocityMotionMagicSetpointNoFOCImpl(velocitySupplier().value(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicNoFOCControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicTorqueCurrentFOCCommand(
    std::function<vel_t()> velocitySupplier) {
    return VelocityMotionMagicTorqueCurrentFOCCommand(velocitySupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicTorqueCurrentFOCCommand(
    std::function<vel_t()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocityMotionMagicTorqueCurrentFOCImpl(velocitySupplier().value(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicTorqueCurrentFOCControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicTorqueCurrentFOCNoEndCommand(
    std::function<vel_t()> velocitySupplier) {
    return VelocityMotionMagicTorqueCurrentFOCNoEndCommand(velocitySupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicTorqueCurrentFOCNoEndCommand(
    std::function<vel_t()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocityMotionMagicTorqueCurrentFOCImpl(velocitySupplier().value(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicTorqueCurrentFOCControl"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocitySetpointIgnoreLimitsCommand(
    std::function<vel_t()> velocitySupplier) {
    return VelocitySetpointIgnoreLimitsCommand(velocitySupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocitySetpointIgnoreLimitsCommand(
    std::function<vel_t()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocitySetpointIgnoreLimitsImpl(velocitySupplier().value(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityControlIgnoreLimits"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocitySetpointNoFOCIgnoreLimitsCommand(
    std::function<vel_t()> velocitySupplier) {
    return VelocitySetpointNoFOCIgnoreLimitsCommand(velocitySupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocitySetpointNoFOCIgnoreLimitsCommand(
    std::function<vel_t()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocitySetpointNoFOCIgnoreLimitsImpl(velocitySupplier().value(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityNoFOCControlIgnoreLimits"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointIgnoreLimitsCommand(
    std::function<vel_t()> velocitySupplier) {
    return VelocityMotionMagicSetpointIgnoreLimitsCommand(velocitySupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointIgnoreLimitsCommand(
    std::function<vel_t()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocityMotionMagicSetpointIgnoreLimitsImpl(velocitySupplier().value(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicControlIgnoreLimits"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointNoFOCIgnoreLimitsCommand(
    std::function<vel_t()> velocitySupplier) {
    return VelocityMotionMagicSetpointNoFOCIgnoreLimitsCommand(velocitySupplier, 0);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::VelocityMotionMagicSetpointNoFOCIgnoreLimitsCommand(
    std::function<vel_t()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocityMotionMagicSetpointNoFOCIgnoreLimitsImpl(velocitySupplier().value(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicNoFOCControlIgnoreLimits"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::SetCoastCommand() {
    return StartEnd(
                [this] { SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue::Coast); },
                [this] { SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue::Brake); })
            .WithName(LogKey("CoastMode"))
            .IgnoringDisable(true);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::SetMotionMagicConfigCommand(
    ctre::phoenix6::configs::MotionMagicConfigs configs) {
    return frc2::cmd::RunOnce([this, configs] { SetMotionMagicConfig(configs); });
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetSupplyCurrentLimit(double amps) {
    SetSupplyCurrentLimitImpl(amps);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::SetSupplyCurrentLimitCommand(double amps) {
    return RunOnce([this, amps] { SetSupplyCurrentLimitImpl(amps); })
            .WithName(LogKey("/setSupplyCurrentLimitCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::SetSupplyCurrentLimitCommand(
    std::function<double()> ampsSupplier) {
    return RunOnce([this, ampsSupplier] { SetSupplyCurrentLimitImpl(ampsSupplier()); })
            .WithName(LogKey("/setSupplyCurrentLimitCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<pos_t, T, U>::SetStatorCurrentLimit(double amps) {
    SetStatorCurrentLimitImpl(amps);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::SetStatorCurrentLimitCommand(double amps) {
    return RunOnce([this, amps] { SetStatorCurrentLimitImpl(amps); })
            .WithName(LogKey("/setStatorCurrentLimitCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::SetStatorCurrentLimitCommand(
    std::function<double()> ampsSupplier) {
    return RunOnce([this, ampsSupplier] { SetStatorCurrentLimitImpl(ampsSupplier()); })
            .WithName(LogKey("/setStatorCurrentLimitCommand"));
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<pos_t, T, U>::SystemTestCommand(
    const std::string& testName, double dutyCycle, double durationSecs) {
    return frc2::cmd::Sequence(
                frc2::cmd::RunOnce([this] { SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue::Coast); }),
                frc2::cmd::Run(
                    [this, testName, dutyCycle] {
                        SetOpenLoopDutyCycleImpl(dutyCycle);
                        akit::Logger::RecordOutput("SystemTest/" + testName + "/StatorCurrent", GetStatorCurrentAmps());
                        akit::Logger::RecordOutput("SystemTest/" + testName + "/SupplyCurrent", GetSupplyCurrentAmps());
                        akit::Logger::RecordOutput("SystemTest/" + testName + "/Velocity",
                                                   GetCurrentVelocity().value());
                    },
                    {this})
                .WithTimeout(units::second_t{durationSecs}),
                frc2::cmd::RunOnce([this] {
                    SetOpenLoopDutyCycleImpl(0.0);
                    SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue::Brake);
                }))
            .WithName("Test " + testName);
}
