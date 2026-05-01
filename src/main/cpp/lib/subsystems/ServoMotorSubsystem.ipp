#pragma once

#include <memory>
#include <utility>

#include "akit/Logger.h"
#include "frc/Timer.h"
#include "frc2/command/Commands.h"
#include "lib/util/Util.h"

template<IsMotorInputs T, IsMotorIO U>
ServoMotorSubsystem<T, U>::ServoMotorSubsystem(const ServoMotorSubsystemConfig &config, T inputs, U *io)
    : frc2::SubsystemBase(config.name)
      , io(io)
      , inputs(std::move(inputs))
      , conf(config) {
    SetDefaultCommand(DutyCycleCommand([] { return 0.0; })
        .WithName(LogKey(" Default Command Neutral"))
        .IgnoringDisable(true));
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::Periodic() {
    double timestamp = static_cast<double>(frc::Timer::GetFPGATimestamp());
    io->ReadInputs(inputs);

    akit::Logger::ProcessInputs(GetName(), inputs);
    akit::Logger::RecordOutput(
        LogKey("/latencyPeriodicSec"),
        static_cast<double>(frc::Timer::GetFPGATimestamp()) - timestamp);
    akit::Logger::RecordOutput(
        LogKey("/currentCommand"),
        GetCurrentCommand() == nullptr ? "Default" : GetCurrentCommand()->GetName());
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetCurrentPosition(double positionUnits) {
    io->SetCurrentPosition(positionUnits);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetCurrentPositionAsZero() {
    io->SetCurrentPositionAsZero();
}

template<IsMotorInputs T, IsMotorIO U>
std::string ServoMotorSubsystem<T, U>::LogKey(std::string_view suffix) {
    auto [it, inserted] = logKeyCache.try_emplace(
        std::string{suffix}, GetName() + std::string{suffix});
    return it->second;
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetOpenLoopDutyCycleImpl(double dutyCycle) {
    akit::Logger::RecordOutput(LogKey("/API/setOpenLoopDutyCycle/dutyCycle"), dutyCycle);
    io->SetOpenLoopDutyCycle(dutyCycle);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetOpenLoopDutyCycleNoFOCImpl(double dutyCycle) {
    akit::Logger::RecordOutput(LogKey("/API/setOpenLoopDutyCycleNoFOC/dutyCycle"), dutyCycle);
    io->SetOpenLoopDutyCycleNoFOC(dutyCycle);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetOpenLoopDutyCycleIgnoreLimitsImpl(double dutyCycle) {
    akit::Logger::RecordOutput(LogKey("/API/setOpenLoopDutyCycleIgnoreLimits/dutyCycle"), dutyCycle);
    io->SetOpenLoopDutyCycleIgnoreLimits(dutyCycle);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetVoltageImpl(double voltage) {
    akit::Logger::RecordOutput(LogKey("/API/setVoltageImpl/voltage"), voltage);
    io->SetVoltageOutput(voltage);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue mode) {
    std::string modeName = "Unknown";
    if (mode == ctre::phoenix6::signals::NeutralModeValue::Coast) {
        modeName = "Coast";
    } else if (mode == ctre::phoenix6::signals::NeutralModeValue::Brake) {
        modeName = "Brake";
    }
    akit::Logger::RecordOutput(LogKey("/API/setNeutralModeImpl/Mode"), modeName);
    io->SetNeutralMode(mode);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetPositionSetpointImpl(double units) {
    positionSetpointUnits = units;
    akit::Logger::RecordOutput(LogKey("/API/setPositionSetpointImp/Units"), units);
    io->SetPositionSetpoint(units);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetPositionTorqueCurrentFOCImpl(double units) {
    positionSetpointUnits = units;
    akit::Logger::RecordOutput(LogKey("/API/setPositionTorqueCurrentFOCImp/Units"), units);
    io->SetPositionTorqueCurrentFOC(units);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetMotionMagicSetpointImpl(double units, int slot) {
    positionSetpointUnits = units;
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImp/Units"), units);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicSetpointImp/Slot"), slot);
    io->SetMotionMagicSetpoint(units, slot);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetMotionMagicSetpointImpl(
    double units, const ctre::phoenix6::configs::MotionMagicConfigs &config, int slot) {
    positionSetpointUnits = units;
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

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetMotionMagicSetpointImpl(
    double units, const ctre::phoenix6::configs::MotionMagicConfigs &config, double feedforward, int slot) {
    positionSetpointUnits = units;
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

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetMotionMagicTorqueCurrentFOCImpl(double units, int slot) {
    positionSetpointUnits = units;
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImp/Units"), units);
    akit::Logger::RecordOutput(LogKey("/API/setMotionMagicTorqueCurrentFOCImp/Slot"), slot);
    io->SetMotionMagicTorqueCurrentFOC(units, slot);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetMotionMagicTorqueCurrentFOCImpl(
    double units, const ctre::phoenix6::configs::MotionMagicConfigs &config, int slot) {
    positionSetpointUnits = units;
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

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetMotionMagicTorqueCurrentFOCImpl(
    double units, const ctre::phoenix6::configs::MotionMagicConfigs &config, double feedforward, int slot) {
    positionSetpointUnits = units;
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

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetVelocitySetpointImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocitySetpointImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocitySetpoint(unitsPerSecond, slot);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetVelocitySetpointNoFOCImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocitySetpointNoFOCImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocitySetpointNoFOC(unitsPerSecond, slot);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetVelocitySetpointIgnoreLimitsImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocitySetpointIgnoreLimitsImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocitySetpointIgnoreLimits(unitsPerSecond, slot);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetVelocitySetpointNoFOCIgnoreLimitsImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocitySetpointNoFOCIgnoreLimitsImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocitySetpointNoFOCIgnoreLimits(unitsPerSecond, slot);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetVelocityMotionMagicSetpointImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicSetpointImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocityMotionMagicSetpoint(unitsPerSecond, slot);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetVelocityMotionMagicSetpointNoFOCImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicSetpointNoFOCImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocityMotionMagicSetpointNoFOC(unitsPerSecond, slot);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetVelocityMotionMagicTorqueCurrentFOCImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicTorqueCurrentFOCImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocityMotionMagicTorqueCurrentFOC(unitsPerSecond, slot);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetVelocityMotionMagicSetpointIgnoreLimitsImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicSetpointIgnoreLimitsImpl/UnitsPerS"), unitsPerSecond);
    io->SetVelocityMotionMagicSetpointIgnoreLimits(unitsPerSecond, slot);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetVelocityMotionMagicSetpointNoFOCIgnoreLimitsImpl(double unitsPerSecond, int slot) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityMotionMagicSetpointNoFOCIgnoreLimitsImpl/UnitsPerS"),
                               unitsPerSecond);
    io->SetVelocityMotionMagicSetpointNoFOCIgnoreLimits(unitsPerSecond, slot);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetTorqueCurrentFOCImpl(double current) {
    akit::Logger::RecordOutput(LogKey("/API/setTorqueCurrentFoC/Current"), current);
    io->SetTorqueCurrentFOC(current);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetVelocityTorqueCurrentFOCImpl(double velocity, double feedforward) {
    akit::Logger::RecordOutput(LogKey("/API/setVelocityTorqueCurrentFOC/Velocity"), velocity);
    akit::Logger::RecordOutput(LogKey("/API/setVelocityTorqueCurrentFOC/Feedforward"), feedforward);
    io->SetVelocityTorqueCurrentFOC(velocity, feedforward);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetSupplyCurrentLimitImpl(double amps) {
    akit::Logger::RecordOutput(LogKey("/API/setSupplyCurrentLimit/Amps"), amps);
    io->SetSupplyCurrentLimit(amps);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetStatorCurrentLimitImpl(double amps) {
    akit::Logger::RecordOutput(LogKey("/API/setStatorCurrentLimit/Amps"), amps);
    io->SetStatorCurrentLimit(amps);
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetMotionMagicConfig(
    const ctre::phoenix6::configs::MotionMagicConfigs &config) {
    io->SetMotionMagicConfig(config);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::WithoutLimitsTemporarily() {
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

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::DutyCycleCommand(std::function<double()> dutyCycleSupplier) {
    return RunEnd(
                [this, dutyCycleSupplier] { SetOpenLoopDutyCycleImpl(dutyCycleSupplier()); },
                [this] { SetOpenLoopDutyCycleImpl(0.0); })
            .WithName(LogKey(" DutyCycleControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::DutyCycleCommandNoEnd(std::function<double()> dutyCycleSupplier) {
    return RunEnd(
                [this, dutyCycleSupplier] { SetOpenLoopDutyCycleImpl(dutyCycleSupplier()); },
                [] {})
            .WithName(LogKey(" DutyCycleControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::DutyCycleIgnoreLimitsCommand(std::function<double()> dutyCycleSupplier) {
    return RunEnd(
                [this, dutyCycleSupplier] { SetOpenLoopDutyCycleIgnoreLimitsImpl(dutyCycleSupplier()); },
                [this] { SetOpenLoopDutyCycleIgnoreLimitsImpl(0.0); })
            .WithName(LogKey(" DutyCycleControlIgnoreLimits"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::DutyCycleNoFOCCommand(std::function<double()> dutyCycleSupplier) {
    return RunEnd(
                [this, dutyCycleSupplier] { SetOpenLoopDutyCycleNoFOCImpl(dutyCycleSupplier()); },
                [this] { SetOpenLoopDutyCycleNoFOCImpl(0.0); })
            .WithName(LogKey(" DutyCycleNoFOCControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::DutyCycleNoFOCCommandNoEnd(std::function<double()> dutyCycleSupplier) {
    return RunEnd(
                [this, dutyCycleSupplier] { SetOpenLoopDutyCycleNoFOCImpl(dutyCycleSupplier()); },
                [] {})
            .WithName(LogKey(" DutyCycleNoFOCControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VoltageCommand(std::function<double()> voltageSupplier) {
    return RunEnd(
                [this, voltageSupplier] { SetVoltageImpl(voltageSupplier()); },
                [this] { SetVoltageImpl(0.0); })
            .WithName(LogKey(" VoltageControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::SetTorqueCurrentFOCCommand(std::function<double()> currentSupplier) {
    return RunEnd(
                [this, currentSupplier] { SetTorqueCurrentFOCImpl(currentSupplier()); },
                [] {})
            .WithName(LogKey(" torqueCurrentFOCCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityTorqueCurrentCommand(std::function<double()> velocitySupplier) {
    return RunEnd(
                [this, velocitySupplier] { SetVelocityTorqueCurrentFOCImpl(velocitySupplier(), 0.0); },
                [] {})
            .WithName(LogKey(" velocityTorqueCurrentCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityTorqueCurrentCommand(
    std::function<double()> velocitySupplier, std::function<double()> feedforwardSupplier) {
    return RunEnd(
                [this, velocitySupplier, feedforwardSupplier] {
                    SetVelocityTorqueCurrentFOCImpl(velocitySupplier(), feedforwardSupplier());
                },
                [] {})
            .WithName(LogKey(" velocityTorqueCurrentCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::PositionSetpointCommand(std::function<double()> unitSupplier) {
    return RunEnd(
                [this, unitSupplier] { SetPositionSetpointImpl(unitSupplier()); },
                [] {})
            .WithName(LogKey(" positionSetpointCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::PositionSetpointUntilOnTargetCommand(
    std::function<double()> unitSupplier, std::function<double()> epsilonSupplier) {
    return PositionSetpointCommand(unitSupplier)
            .Until([this, unitSupplier, epsilonSupplier] {
                return Util::EpsilonEquals(unitSupplier(), inputs.unitPosition, epsilonSupplier());
            });
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::PositionTorqueCurrentFOCCommand(std::function<double()> unitSupplier) {
    return RunEnd(
                [this, unitSupplier] { SetPositionTorqueCurrentFOCImpl(unitSupplier()); },
                [] {})
            .WithName(LogKey(" positionTorqueCurrentFOCCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicSetpointCommand(std::function<double()> unitSupplier) {
    return MotionMagicSetpointCommand(unitSupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicSetpointCommand(std::function<double()> unitSupplier, int slot) {
    return RunEnd(
                [this, unitSupplier, slot] { SetMotionMagicSetpointImpl(unitSupplier(), slot); },
                [] {})
            .WithName(LogKey(" motionMagicSetpointCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicSetpointCommand(
    std::function<double()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier) {
    return MotionMagicSetpointCommand(unitSupplier, configSupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicSetpointCommand(
    std::function<double()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    int slot) {
    return RunEnd(
                [this, unitSupplier, configSupplier, slot] {
                    SetMotionMagicSetpointImpl(unitSupplier(), configSupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" dynamicMotionMagicSetpointCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicSetpointCommand(
    std::function<double()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    std::function<double()> feedforwardSupplier,
    int slot) {
    return RunEnd(
                [this, unitSupplier, configSupplier, feedforwardSupplier, slot] {
                    SetMotionMagicSetpointImpl(unitSupplier(), configSupplier(), feedforwardSupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" dynamicMotionMagicSetpointCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicSetpointCommandBlocking(
    std::function<double()> setpointSupplier, double tolerance) {
    return MotionMagicSetpointCommandBlocking(setpointSupplier, tolerance, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicSetpointCommandBlocking(
    std::function<double()> setpointSupplier, double tolerance, int slot) {
    return MotionMagicSetpointCommand([setpointSupplier] { return setpointSupplier(); }, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(GetCurrentPosition(), setpointSupplier(), tolerance);
            });
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicSetpointCommandBlocking(
    std::function<double()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    double tolerance) {
    return MotionMagicSetpointCommandBlocking(setpointSupplier, configSupplier, tolerance, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicSetpointCommandBlocking(
    std::function<double()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    double tolerance,
    int slot) {
    return MotionMagicSetpointCommand(setpointSupplier, configSupplier, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(GetCurrentPosition(), setpointSupplier(), tolerance);
            });
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicSetpointCommandBlocking(
    std::function<double()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    std::function<double()> feedforwardSupplier,
    double tolerance,
    int slot) {
    return MotionMagicSetpointCommand(setpointSupplier, configSupplier, feedforwardSupplier, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(GetCurrentPosition(), setpointSupplier(), tolerance);
            });
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicTorqueCurrentFOCCommand(std::function<double()> unitSupplier) {
    return MotionMagicTorqueCurrentFOCCommand(unitSupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicTorqueCurrentFOCCommand(
    std::function<double()> unitSupplier, int slot) {
    return RunEnd(
                [this, unitSupplier, slot] { SetMotionMagicTorqueCurrentFOCImpl(unitSupplier(), slot); },
                [] {})
            .WithName(LogKey(" motionMagicTorqueCurrentFOCCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicTorqueCurrentFOCCommand(
    std::function<double()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier) {
    return MotionMagicTorqueCurrentFOCCommand(unitSupplier, configSupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicTorqueCurrentFOCCommand(
    std::function<double()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    int slot) {
    return RunEnd(
                [this, unitSupplier, configSupplier, slot] {
                    SetMotionMagicTorqueCurrentFOCImpl(unitSupplier(), configSupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" dynamicMotionMagicTorqueCurrentFOCCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicTorqueCurrentFOCCommand(
    std::function<double()> unitSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    std::function<double()> feedforwardSupplier,
    int slot) {
    return RunEnd(
                [this, unitSupplier, configSupplier, feedforwardSupplier, slot] {
                    SetMotionMagicTorqueCurrentFOCImpl(unitSupplier(), configSupplier(), feedforwardSupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" dynamicMotionMagicTorqueCurrentFOCCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicTorqueCurrentFOCCommandBlocking(
    std::function<double()> setpointSupplier, double tolerance) {
    return MotionMagicTorqueCurrentFOCCommandBlocking(setpointSupplier, tolerance, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicTorqueCurrentFOCCommandBlocking(
    std::function<double()> setpointSupplier, double tolerance, int slot) {
    return MotionMagicTorqueCurrentFOCCommand([setpointSupplier] { return setpointSupplier(); }, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(GetCurrentPosition(), setpointSupplier(), tolerance);
            });
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicTorqueCurrentFOCCommandBlocking(
    std::function<double()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    double tolerance) {
    return MotionMagicTorqueCurrentFOCCommandBlocking(setpointSupplier, configSupplier, tolerance, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicTorqueCurrentFOCCommandBlocking(
    std::function<double()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    double tolerance,
    int slot) {
    return MotionMagicTorqueCurrentFOCCommand(setpointSupplier, configSupplier, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(GetCurrentPosition(), setpointSupplier(), tolerance);
            });
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::MotionMagicTorqueCurrentFOCCommandBlocking(
    std::function<double()> setpointSupplier,
    std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
    std::function<double()> feedforwardSupplier,
    double tolerance,
    int slot) {
    return MotionMagicTorqueCurrentFOCCommand(setpointSupplier, configSupplier, feedforwardSupplier, slot)
            .Until([this, setpointSupplier, tolerance] {
                return Util::EpsilonEquals(GetCurrentPosition(), setpointSupplier(), tolerance);
            });
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocitySetpointCommand(std::function<double()> velocitySupplier) {
    return VelocitySetpointCommand(velocitySupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocitySetpointCommand(
    std::function<double()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] { SetVelocitySetpointImpl(velocitySupplier(), slot); },
                [] {})
            .WithName(LogKey(" VelocityControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocitySetpointNoFOCCommand(std::function<double()> velocitySupplier) {
    return VelocitySetpointNoFOCCommand(velocitySupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocitySetpointNoFOCCommand(
    std::function<double()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] { SetVelocitySetpointNoFOCImpl(velocitySupplier(), slot); },
                [] {})
            .WithName(LogKey(" VelocityNoFOCControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicSetpointCommand(
    std::function<double()> velocitySupplier) {
    return VelocityMotionMagicSetpointCommand(velocitySupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicSetpointCommand(
    std::function<double()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] { SetVelocityMotionMagicSetpointImpl(velocitySupplier(), slot); },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicSetpointNoEndCommand(
    std::function<double()> velocitySupplier) {
    return VelocityMotionMagicSetpointNoEndCommand(velocitySupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicSetpointNoEndCommand(
    std::function<double()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] { SetVelocityMotionMagicSetpointImpl(velocitySupplier(), slot); },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicSetpointNoFOCNoEndCommand(
    std::function<double()> velocitySupplier) {
    return VelocityMotionMagicSetpointNoFOCNoEndCommand(velocitySupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicSetpointNoFOCNoEndCommand(
    std::function<double()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] { SetVelocityMotionMagicSetpointNoFOCImpl(velocitySupplier(), slot); },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicNoFOCControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicTorqueCurrentFOCCommand(
    std::function<double()> velocitySupplier) {
    return VelocityMotionMagicTorqueCurrentFOCCommand(velocitySupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicTorqueCurrentFOCCommand(
    std::function<double()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocityMotionMagicTorqueCurrentFOCImpl(velocitySupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicTorqueCurrentFOCControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicTorqueCurrentFOCNoEndCommand(
    std::function<double()> velocitySupplier) {
    return VelocityMotionMagicTorqueCurrentFOCNoEndCommand(velocitySupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicTorqueCurrentFOCNoEndCommand(
    std::function<double()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocityMotionMagicTorqueCurrentFOCImpl(velocitySupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicTorqueCurrentFOCControl"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocitySetpointIgnoreLimitsCommand(
    std::function<double()> velocitySupplier) {
    return VelocitySetpointIgnoreLimitsCommand(velocitySupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocitySetpointIgnoreLimitsCommand(
    std::function<double()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] { SetVelocitySetpointIgnoreLimitsImpl(velocitySupplier(), slot); },
                [] {})
            .WithName(LogKey(" VelocityControlIgnoreLimits"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocitySetpointNoFOCIgnoreLimitsCommand(
    std::function<double()> velocitySupplier) {
    return VelocitySetpointNoFOCIgnoreLimitsCommand(velocitySupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocitySetpointNoFOCIgnoreLimitsCommand(
    std::function<double()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] { SetVelocitySetpointNoFOCIgnoreLimitsImpl(velocitySupplier(), slot); },
                [] {})
            .WithName(LogKey(" VelocityNoFOCControlIgnoreLimits"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicSetpointIgnoreLimitsCommand(
    std::function<double()> velocitySupplier) {
    return VelocityMotionMagicSetpointIgnoreLimitsCommand(velocitySupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicSetpointIgnoreLimitsCommand(
    std::function<double()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocityMotionMagicSetpointIgnoreLimitsImpl(velocitySupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicControlIgnoreLimits"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicSetpointNoFOCIgnoreLimitsCommand(
    std::function<double()> velocitySupplier) {
    return VelocityMotionMagicSetpointNoFOCIgnoreLimitsCommand(velocitySupplier, 0);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::VelocityMotionMagicSetpointNoFOCIgnoreLimitsCommand(
    std::function<double()> velocitySupplier, int slot) {
    return RunEnd(
                [this, velocitySupplier, slot] {
                    SetVelocityMotionMagicSetpointNoFOCIgnoreLimitsImpl(velocitySupplier(), slot);
                },
                [] {})
            .WithName(LogKey(" VelocityMotionMagicNoFOCControlIgnoreLimits"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::SetCoastCommand() {
    return StartEnd(
                [this] { SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue::Coast); },
                [this] { SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue::Brake); })
            .WithName(LogKey("CoastMode"))
            .IgnoringDisable(true);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::SetMotionMagicConfigCommand(
    ctre::phoenix6::configs::MotionMagicConfigs configs) {
    return frc2::cmd::RunOnce([this, configs] { SetMotionMagicConfig(configs); });
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetSupplyCurrentLimit(double amps) {
    SetSupplyCurrentLimitImpl(amps);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::SetSupplyCurrentLimitCommand(double amps) {
    return RunOnce([this, amps] { SetSupplyCurrentLimitImpl(amps); })
            .WithName(LogKey("/setSupplyCurrentLimitCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::SetSupplyCurrentLimitCommand(std::function<double()> ampsSupplier) {
    return RunOnce([this, ampsSupplier] { SetSupplyCurrentLimitImpl(ampsSupplier()); })
            .WithName(LogKey("/setSupplyCurrentLimitCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystem<T, U>::SetStatorCurrentLimit(double amps) {
    SetStatorCurrentLimitImpl(amps);
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::SetStatorCurrentLimitCommand(double amps) {
    return RunOnce([this, amps] { SetStatorCurrentLimitImpl(amps); })
            .WithName(LogKey("/setStatorCurrentLimitCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::SetStatorCurrentLimitCommand(std::function<double()> ampsSupplier) {
    return RunOnce([this, ampsSupplier] { SetStatorCurrentLimitImpl(ampsSupplier()); })
            .WithName(LogKey("/setStatorCurrentLimitCommand"));
}

template<IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystem<T, U>::SystemTestCommand(
    const std::string &testName, double dutyCycle, double durationSecs) {
    return frc2::cmd::Sequence(
                frc2::cmd::RunOnce([this] { SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue::Coast); }),
                frc2::cmd::Run(
                    [this, testName, dutyCycle] {
                        SetOpenLoopDutyCycleImpl(dutyCycle);
                        akit::Logger::RecordOutput("SystemTest/" + testName + "/StatorCurrent", GetStatorCurrentAmps());
                        akit::Logger::RecordOutput("SystemTest/" + testName + "/SupplyCurrent", GetSupplyCurrentAmps());
                        akit::Logger::RecordOutput("SystemTest/" + testName + "/Velocity", GetCurrentVelocity());
                    },
                    {this})
                .WithTimeout(units::second_t{durationSecs}),
                frc2::cmd::RunOnce([this] {
                    SetOpenLoopDutyCycleImpl(0.0);
                    SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue::Brake);
                }))
            .WithName("Test " + testName);
}
