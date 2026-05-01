#pragma once

#include <concepts>
#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/configs/MotionMagicConfigs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "lib/subsystems/MotorIO.h"
#include "lib/subsystems/MotorInputs.h"
#include "lib/subsystems/ServoMotorSubsystemConfig.h"

template<typename T>
concept IsMotorInputs = std::derived_from<T, MotorInputs>;

template<typename T>
concept IsMotorIO = std::derived_from<T, MotorIO>;

template<IsMotorInputs T, IsMotorIO U>
class ServoMotorSubsystem : public frc2::SubsystemBase {
public:
    ServoMotorSubsystem(const ServoMotorSubsystemConfig &config, T inputs, U *io);

    void Periodic() override;

    virtual double GetCurrentPosition() const { return inputs.unitPosition; }
    virtual double GetCurrentVelocity() const { return inputs.velocityUnitsPerSecond; }
    virtual double GetSupplyCurrentAmps() const { return inputs.currentSupplyAmps; }
    virtual double GetStatorCurrentAmps() const { return inputs.currentStatorAmps; }
    double GetAppliedVolts() const { return inputs.appliedVolts; }
    virtual double GetAverageStatorCurrentAmps() const { return GetStatorCurrentAmps(); }
    virtual double GetAverageSupplyCurrentAmps() const { return GetSupplyCurrentAmps(); }
    double GetPositionSetpointUnits() const { return positionSetpointUnits; }

    virtual void SetCurrentPosition(double positionUnits);
    virtual void SetCurrentPositionAsZero();

protected:
    std::string LogKey(std::string_view suffix);

    void SetOpenLoopDutyCycleImpl(double dutyCycle);
    void SetOpenLoopDutyCycleNoFOCImpl(double dutyCycle);
    void SetOpenLoopDutyCycleIgnoreLimitsImpl(double dutyCycle);
    void SetVoltageImpl(double voltage);
    void SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue mode);
    void SetPositionSetpointImpl(double units);
    void SetPositionTorqueCurrentFOCImpl(double units);
    void SetMotionMagicSetpointImpl(double units, int slot);
    void SetMotionMagicSetpointImpl(
        double units,
        const ctre::phoenix6::configs::MotionMagicConfigs &config,
        int slot);
    void SetMotionMagicSetpointImpl(
        double units,
        const ctre::phoenix6::configs::MotionMagicConfigs &config,
        double feedforward,
        int slot);
    void SetMotionMagicTorqueCurrentFOCImpl(double units, int slot);
    void SetMotionMagicTorqueCurrentFOCImpl(
        double units,
        const ctre::phoenix6::configs::MotionMagicConfigs &config,
        int slot);
    void SetMotionMagicTorqueCurrentFOCImpl(
        double units,
        const ctre::phoenix6::configs::MotionMagicConfigs &config,
        double feedforward,
        int slot);
    void SetVelocitySetpointImpl(double unitsPerSecond, int slot);
    void SetVelocitySetpointNoFOCImpl(double unitsPerSecond, int slot);
    void SetVelocitySetpointIgnoreLimitsImpl(double unitsPerSecond, int slot);
    void SetVelocitySetpointNoFOCIgnoreLimitsImpl(double unitsPerSecond, int slot);
    void SetVelocityMotionMagicSetpointImpl(double unitsPerSecond, int slot);
    void SetVelocityMotionMagicSetpointNoFOCImpl(double unitsPerSecond, int slot);
    void SetVelocityMotionMagicTorqueCurrentFOCImpl(double unitsPerSecond, int slot);
    void SetVelocityMotionMagicSetpointIgnoreLimitsImpl(double unitsPerSecond, int slot);
    void SetVelocityMotionMagicSetpointNoFOCIgnoreLimitsImpl(
        double unitsPerSecond, int slot);
    void SetTorqueCurrentFOCImpl(double current);
    void SetVelocityTorqueCurrentFOCImpl(double velocity, double feedforward);
    void SetSupplyCurrentLimitImpl(double amps);
    void SetStatorCurrentLimitImpl(double amps);
    void SetMotionMagicConfig(
        const ctre::phoenix6::configs::MotionMagicConfigs &config);
    frc2::CommandPtr WithoutLimitsTemporarily();

    U *io;
    T inputs;
    double positionSetpointUnits = 0.0;
    ServoMotorSubsystemConfig conf;

public:
    frc2::CommandPtr DutyCycleCommand(std::function<double()> dutyCycleSupplier);
    frc2::CommandPtr DutyCycleCommandNoEnd(std::function<double()> dutyCycleSupplier);
    frc2::CommandPtr DutyCycleIgnoreLimitsCommand(std::function<double()> dutyCycleSupplier);
    frc2::CommandPtr DutyCycleNoFOCCommand(std::function<double()> dutyCycleSupplier);
    frc2::CommandPtr DutyCycleNoFOCCommandNoEnd(std::function<double()> dutyCycleSupplier);

    frc2::CommandPtr VoltageCommand(std::function<double()> voltageSupplier);

    frc2::CommandPtr SetTorqueCurrentFOCCommand(std::function<double()> currentSupplier);
    frc2::CommandPtr VelocityTorqueCurrentCommand(std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocityTorqueCurrentCommand(
        std::function<double()> velocitySupplier,
        std::function<double()> feedforwardSupplier);

    frc2::CommandPtr PositionSetpointCommand(std::function<double()> unitSupplier);
    frc2::CommandPtr PositionSetpointUntilOnTargetCommand(
        std::function<double()> unitSupplier, std::function<double()> epsilonSupplier);
    frc2::CommandPtr PositionTorqueCurrentFOCCommand(std::function<double()> unitSupplier);

    frc2::CommandPtr MotionMagicSetpointCommand(std::function<double()> unitSupplier);
    frc2::CommandPtr MotionMagicSetpointCommand(std::function<double()> unitSupplier, int slot);
    frc2::CommandPtr MotionMagicSetpointCommand(
        std::function<double()> unitSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier);
    frc2::CommandPtr MotionMagicSetpointCommand(
        std::function<double()> unitSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
        int slot);
    frc2::CommandPtr MotionMagicSetpointCommand(
        std::function<double()> unitSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
        std::function<double()> feedforwardSupplier,
        int slot);
    frc2::CommandPtr MotionMagicSetpointCommandBlocking(
        std::function<double()> setpointSupplier, double tolerance);
    frc2::CommandPtr MotionMagicSetpointCommandBlocking(
        std::function<double()> setpointSupplier, double tolerance, int slot);
    frc2::CommandPtr MotionMagicSetpointCommandBlocking(
        std::function<double()> setpointSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
        double tolerance);
    frc2::CommandPtr MotionMagicSetpointCommandBlocking(
        std::function<double()> setpointSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
        double tolerance,
        int slot);
    frc2::CommandPtr MotionMagicSetpointCommandBlocking(
        std::function<double()> setpointSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
        std::function<double()> feedforwardSupplier,
        double tolerance,
        int slot);

    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommand(std::function<double()> unitSupplier);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommand(
        std::function<double()> unitSupplier, int slot);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommand(
        std::function<double()> unitSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommand(
        std::function<double()> unitSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
        int slot);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommand(
        std::function<double()> unitSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
        std::function<double()> feedforwardSupplier,
        int slot);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommandBlocking(
        std::function<double()> setpointSupplier, double tolerance);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommandBlocking(
        std::function<double()> setpointSupplier, double tolerance, int slot);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommandBlocking(
        std::function<double()> setpointSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
        double tolerance);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommandBlocking(
        std::function<double()> setpointSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
        double tolerance,
        int slot);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommandBlocking(
        std::function<double()> setpointSupplier,
        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
        std::function<double()> feedforwardSupplier,
        double tolerance,
        int slot);

    frc2::CommandPtr VelocitySetpointCommand(std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocitySetpointCommand(std::function<double()> velocitySupplier, int slot);
    frc2::CommandPtr VelocitySetpointNoFOCCommand(std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocitySetpointNoFOCCommand(
        std::function<double()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicSetpointCommand(
        std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicSetpointCommand(
        std::function<double()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicSetpointNoEndCommand(
        std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicSetpointNoEndCommand(
        std::function<double()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicSetpointNoFOCNoEndCommand(
        std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicSetpointNoFOCNoEndCommand(
        std::function<double()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicTorqueCurrentFOCCommand(
        std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicTorqueCurrentFOCCommand(
        std::function<double()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicTorqueCurrentFOCNoEndCommand(
        std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicTorqueCurrentFOCNoEndCommand(
        std::function<double()> velocitySupplier, int slot);
    frc2::CommandPtr VelocitySetpointIgnoreLimitsCommand(
        std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocitySetpointIgnoreLimitsCommand(
        std::function<double()> velocitySupplier, int slot);
    frc2::CommandPtr VelocitySetpointNoFOCIgnoreLimitsCommand(
        std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocitySetpointNoFOCIgnoreLimitsCommand(
        std::function<double()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicSetpointIgnoreLimitsCommand(
        std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicSetpointIgnoreLimitsCommand(
        std::function<double()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicSetpointNoFOCIgnoreLimitsCommand(
        std::function<double()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicSetpointNoFOCIgnoreLimitsCommand(
        std::function<double()> velocitySupplier, int slot);

    frc2::CommandPtr SetCoastCommand();
    frc2::CommandPtr SetMotionMagicConfigCommand(
        ctre::phoenix6::configs::MotionMagicConfigs configs);

    void SetSupplyCurrentLimit(double amps);
    frc2::CommandPtr SetSupplyCurrentLimitCommand(double amps);
    frc2::CommandPtr SetSupplyCurrentLimitCommand(std::function<double()> ampsSupplier);

    void SetStatorCurrentLimit(double amps);
    frc2::CommandPtr SetStatorCurrentLimitCommand(double amps);
    frc2::CommandPtr SetStatorCurrentLimitCommand(std::function<double()> ampsSupplier);

    frc2::CommandPtr SystemTestCommand(
        const std::string &testName, double dutyCycle, double durationSecs);

private:
    std::unordered_map<std::string, std::string> logKeyCache;
};

#include "lib/subsystems/ServoMotorSubsystem.ipp"
