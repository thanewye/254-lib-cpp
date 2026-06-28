#pragma once

#include <concepts>
#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>

#include <ctre/phoenix6/configs/MotionMagicConfigs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/time.h>

#include "lib/subsystems/MotorIO.h"
#include "lib/subsystems/MotorInputs.h"
#include "lib/subsystems/ServoMotorSubsystemConfig.h"

template<typename T>
concept IsMotorInputs = std::derived_from<T, MotorInputs>;

template<typename T>
concept IsMotorIO = std::derived_from<T, MotorIO>;

template<typename pos_t, IsMotorInputs T, IsMotorIO U> class ServoMotorSubsystem : public frc2::SubsystemBase {
public:
    using vel_unit = units::compound_unit<typename pos_t::unit_type, units::inverse<units::seconds>>;
    using vel_t = units::unit_t<vel_unit, typename pos_t::underlying_type>;

    ServoMotorSubsystem(const ServoMotorSubsystemConfig<pos_t>& config, T inputs, U* io);

    void Periodic() override;

    [[nodiscard]] virtual pos_t GetCurrentPosition() const { return pos_t{inputs.unitPosition}; }
    [[nodiscard]] virtual vel_t GetCurrentVelocity() const { return vel_t{inputs.velocityUnitsPerSecond}; }
    [[nodiscard]] virtual double GetSupplyCurrentAmps() const { return inputs.currentSupplyAmps; }
    [[nodiscard]] virtual double GetStatorCurrentAmps() const { return inputs.currentStatorAmps; }
    [[nodiscard]] double GetAppliedVolts() const { return inputs.appliedVolts; }
    [[nodiscard]] virtual double GetAverageStatorCurrentAmps() const { return GetStatorCurrentAmps(); }
    [[nodiscard]] virtual double GetAverageSupplyCurrentAmps() const { return GetSupplyCurrentAmps(); }
    [[nodiscard]] pos_t GetPositionSetpoint() const { return positionSetpoint; }

    virtual void SetCurrentPosition(pos_t position);
    virtual void SetCurrentPositionAsZero();

    frc2::CommandPtr DutyCycleCommand(std::function<double()> dutyCycleSupplier);
    frc2::CommandPtr DutyCycleCommandNoEnd(std::function<double()> dutyCycleSupplier);
    frc2::CommandPtr DutyCycleIgnoreLimitsCommand(std::function<double()> dutyCycleSupplier);
    frc2::CommandPtr DutyCycleNoFOCCommand(std::function<double()> dutyCycleSupplier);
    frc2::CommandPtr DutyCycleNoFOCCommandNoEnd(std::function<double()> dutyCycleSupplier);

    frc2::CommandPtr VoltageCommand(std::function<double()> voltageSupplier);

    frc2::CommandPtr SetTorqueCurrentFOCCommand(std::function<double()> currentSupplier);
    frc2::CommandPtr VelocityTorqueCurrentCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocityTorqueCurrentCommand(std::function<vel_t()> velocitySupplier, std::function<double()> feedforwardSupplier);

    frc2::CommandPtr PositionSetpointCommand(std::function<pos_t()> unitSupplier);
    frc2::CommandPtr PositionSetpointUntilOnTargetCommand(std::function<pos_t()> unitSupplier, std::function<pos_t()> epsilonSupplier);
    frc2::CommandPtr PositionTorqueCurrentFOCCommand(std::function<pos_t()> unitSupplier);

    frc2::CommandPtr MotionMagicSetpointCommand(std::function<pos_t()> unitSupplier);
    frc2::CommandPtr MotionMagicSetpointCommand(std::function<pos_t()> unitSupplier, int slot);
    frc2::CommandPtr MotionMagicSetpointCommand(std::function<pos_t()> unitSupplier,
                                                std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier);
    frc2::CommandPtr MotionMagicSetpointCommand(std::function<pos_t()> unitSupplier,
                                                std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier, int slot);
    frc2::CommandPtr MotionMagicSetpointCommand(std::function<pos_t()> unitSupplier,
                                                std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
                                                std::function<double()> feedforwardSupplier, int slot);
    frc2::CommandPtr MotionMagicSetpointCommandBlocking(std::function<pos_t()> setpointSupplier, pos_t tolerance);
    frc2::CommandPtr MotionMagicSetpointCommandBlocking(std::function<pos_t()> setpointSupplier, pos_t tolerance, int slot);
    frc2::CommandPtr MotionMagicSetpointCommandBlocking(std::function<pos_t()> setpointSupplier,
                                                        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier, pos_t tolerance);
    frc2::CommandPtr MotionMagicSetpointCommandBlocking(std::function<pos_t()> setpointSupplier,
                                                        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier, pos_t tolerance, int slot);
    frc2::CommandPtr MotionMagicSetpointCommandBlocking(std::function<pos_t()> setpointSupplier,
                                                        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
                                                        std::function<double()> feedforwardSupplier, pos_t tolerance, int slot);

    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommand(std::function<pos_t()> unitSupplier);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommand(std::function<pos_t()> unitSupplier, int slot);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommand(std::function<pos_t()> unitSupplier,
                                                        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommand(std::function<pos_t()> unitSupplier,
                                                        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier, int slot);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommand(std::function<pos_t()> unitSupplier,
                                                        std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
                                                        std::function<double()> feedforwardSupplier, int slot);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommandBlocking(std::function<pos_t()> setpointSupplier, pos_t tolerance);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommandBlocking(std::function<pos_t()> setpointSupplier, pos_t tolerance, int slot);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommandBlocking(std::function<pos_t()> setpointSupplier,
                                                                std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier, pos_t tolerance);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommandBlocking(std::function<pos_t()> setpointSupplier,
                                                                std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier, pos_t tolerance,
                                                                int slot);
    frc2::CommandPtr MotionMagicTorqueCurrentFOCCommandBlocking(std::function<pos_t()> setpointSupplier,
                                                                std::function<ctre::phoenix6::configs::MotionMagicConfigs()> configSupplier,
                                                                std::function<double()> feedforwardSupplier, pos_t tolerance, int slot);

    frc2::CommandPtr VelocitySetpointCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocitySetpointCommand(std::function<vel_t()> velocitySupplier, int slot);
    frc2::CommandPtr VelocitySetpointNoFOCCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocitySetpointNoFOCCommand(std::function<vel_t()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicSetpointCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicSetpointCommand(std::function<vel_t()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicSetpointCommand(std::function<vel_t()> velocitySupplier, std::function<double()> feedforwardSupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicSetpointNoEndCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicSetpointNoEndCommand(std::function<vel_t()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicSetpointNoEndCommand(std::function<vel_t()> velocitySupplier, int slot, std::function<double()> feedforwardSupplier);
    frc2::CommandPtr VelocityMotionMagicSetpointNoFOCNoEndCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicSetpointNoFOCNoEndCommand(std::function<vel_t()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicTorqueCurrentFOCCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicTorqueCurrentFOCCommand(std::function<vel_t()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicTorqueCurrentFOCNoEndCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicTorqueCurrentFOCNoEndCommand(std::function<vel_t()> velocitySupplier, int slot);
    frc2::CommandPtr VelocitySetpointIgnoreLimitsCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocitySetpointIgnoreLimitsCommand(std::function<vel_t()> velocitySupplier, int slot);
    frc2::CommandPtr VelocitySetpointNoFOCIgnoreLimitsCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocitySetpointNoFOCIgnoreLimitsCommand(std::function<vel_t()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicSetpointIgnoreLimitsCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicSetpointIgnoreLimitsCommand(std::function<vel_t()> velocitySupplier, int slot);
    frc2::CommandPtr VelocityMotionMagicSetpointNoFOCIgnoreLimitsCommand(std::function<vel_t()> velocitySupplier);
    frc2::CommandPtr VelocityMotionMagicSetpointNoFOCIgnoreLimitsCommand(std::function<vel_t()> velocitySupplier, int slot);

    frc2::CommandPtr SetCoastCommand();
    frc2::CommandPtr SetMotionMagicConfigCommand(ctre::phoenix6::configs::MotionMagicConfigs configs);

    void SetSupplyCurrentLimit(double amps);
    frc2::CommandPtr SetSupplyCurrentLimitCommand(double amps);
    frc2::CommandPtr SetSupplyCurrentLimitCommand(std::function<double()> ampsSupplier);

    void SetStatorCurrentLimit(double amps);
    frc2::CommandPtr SetStatorCurrentLimitCommand(double amps);
    frc2::CommandPtr SetStatorCurrentLimitCommand(std::function<double()> ampsSupplier);

    frc2::CommandPtr SystemTestCommand(const std::string& testName, double dutyCycle, double durationSecs);

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
    void SetMotionMagicSetpointImpl(double units, const ctre::phoenix6::configs::MotionMagicConfigs& config, int slot);
    void SetMotionMagicSetpointImpl(double units, const ctre::phoenix6::configs::MotionMagicConfigs& config, double feedforward, int slot);
    void SetMotionMagicTorqueCurrentFOCImpl(double units, int slot);
    void SetMotionMagicTorqueCurrentFOCImpl(double units, const ctre::phoenix6::configs::MotionMagicConfigs& config, int slot);
    void SetMotionMagicTorqueCurrentFOCImpl(double units, const ctre::phoenix6::configs::MotionMagicConfigs& config, double feedforward, int slot);
    void SetVelocitySetpointImpl(double unitsPerSecond, int slot);
    void SetVelocitySetpointNoFOCImpl(double unitsPerSecond, int slot);
    void SetVelocitySetpointIgnoreLimitsImpl(double unitsPerSecond, int slot);
    void SetVelocitySetpointNoFOCIgnoreLimitsImpl(double unitsPerSecond, int slot);
    void SetVelocityMotionMagicSetpointImpl(double unitsPerSecond, int slot);
    void SetVelocityMotionMagicSetpointImpl(double unitsPerSecond, int slot, double feedforward);
    void SetVelocityMotionMagicSetpointNoFOCImpl(double unitsPerSecond, int slot);
    void SetVelocityMotionMagicTorqueCurrentFOCImpl(double unitsPerSecond, int slot);
    void SetVelocityMotionMagicSetpointIgnoreLimitsImpl(double unitsPerSecond, int slot);
    void SetVelocityMotionMagicSetpointNoFOCIgnoreLimitsImpl(double unitsPerSecond, int slot);
    void SetTorqueCurrentFOCImpl(double current);
    void SetVelocityTorqueCurrentFOCImpl(double velocity, double feedforward);
    void SetSupplyCurrentLimitImpl(double amps);
    void SetStatorCurrentLimitImpl(double amps);
    void SetMotionMagicConfig(const ctre::phoenix6::configs::MotionMagicConfigs& config);
    frc2::CommandPtr WithoutLimitsTemporarily();

    U* io;
    T inputs;
    pos_t positionSetpoint{};
    bool isLoadSheddingActive = false;
    ServoMotorSubsystemConfig<pos_t> conf;
    double defaultSupplyCurrentLimit;

private:
    std::unordered_map<std::string, std::string> logKeyCache;
};

#include "lib/subsystems/ServoMotorSubsystem.ipp"
