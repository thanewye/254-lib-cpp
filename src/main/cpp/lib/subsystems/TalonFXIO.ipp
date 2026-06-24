#pragma once

#include "lib/util/CANStatusLogger.h"
#include "lib/util/CtreUtil.h"
#include "lib/util/StatusSignalManager.h"

template<typename pos_t>
TalonFXIO<pos_t>::TalonFXIO(const ServoMotorSubsystemConfig<pos_t>& config)
    : talon(config.talonCANID.GetDeviceNumber(), config.talonCANID.GetBus())
      , config(config)
      , positionSignal(talon.GetPosition())
      , velocitySignal(talon.GetVelocity())
      , voltageSignal(talon.GetMotorVoltage())
      , currentStatorSignal(talon.GetStatorCurrent())
      , currentSupplySignal(talon.GetSupplyCurrent())
      , rawRotorPositionSignal(talon.GetRotorPosition())
      , temperatureSignal(talon.GetDeviceTemp())
      , signals{&positionSignal, &velocitySignal, &voltageSignal, &currentStatorSignal, &currentSupplySignal, &rawRotorPositionSignal, &temperatureSignal} {
    ctre_util::ConfigureTalonFX(talon, config.fxConfig);
    ctre_util::TryUntilOk(
        [&] {
            return ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
                units::frequency::hertz_t{config.updateFrequencyHz}, signals);
        }, talon.GetDeviceID());
    ctre_util::TryUntilOk(
        [&] { return talon.OptimizeBusUtilization(); }, talon.GetDeviceID());
    CANStatusLogger::GetInstance().RegisterTalonFX(config.name, &talon, config.talonCANID);
    for (auto* sig : signals) {
        StatusSignalManager::GetInstance().Register(sig);
    }
}

template<typename pos_t>
void TalonFXIO<pos_t>::ReadInputs(MotorInputs& inputs) {
    inputs.unitPosition = RotorToUnits(positionSignal.GetValueAsDouble());
    inputs.velocityUnitsPerSecond = RotorToUnits(velocitySignal.GetValueAsDouble());
    inputs.appliedVolts = voltageSignal.GetValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.GetValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.GetValueAsDouble();
    inputs.rawRotorPosition = rawRotorPositionSignal.GetValueAsDouble();
    inputs.motorTempCelsius = temperatureSignal.GetValueAsDouble();
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetOpenLoopDutyCycle(double dutyCycle) {
    talon.SetControl(dutyCycleControl.WithOutput(dutyCycle));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetOpenLoopDutyCycleNoFOC(double dutyCycle) {
    talon.SetControl(dutyCycleControlNoFOC.WithOutput(dutyCycle));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetOpenLoopDutyCycleIgnoreLimits(double dutyCycle) {
    talon.SetControl(dutyCycleControlIgnoreLimits.WithOutput(dutyCycle));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetPositionSetpoint(double units) {
    talon.SetControl(positionVoltageControl.WithPosition(units::angle::turn_t{ClampPosition(units)}));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetMotionMagicSetpoint(double units, int slot) {
    talon.SetControl(
        motionMagicPositionControl.WithPosition(units::angle::turn_t{ClampPosition(units)}).WithSlot(slot));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetMotionMagicSetpoint(double units, double velocity, double acceleration, double jerk,
                                              int slot, double feedforward) {
    talon.SetControl(dynamicMotionMagicVoltage
        .WithPosition(units::angle::turn_t{ClampPosition(units)})
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(velocity)})
        .WithAcceleration(units::angular_acceleration::turns_per_second_squared_t{UnitsToRotor(acceleration)})
        .WithJerk(units::angular_jerk::turns_per_second_cubed_t{UnitsToRotor(jerk)})
        .WithSlot(slot)
        .WithFeedForward(units::voltage::volt_t{feedforward}));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode) {
    config.fxConfig.MotorOutput.NeutralMode = mode;
    talon.SetNeutralMode(mode);
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVelocitySetpoint(double unitsPerSecond, int slot) {
    talon.SetControl(
        velocityVoltageControl.WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)}).
        WithSlot(slot));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVelocitySetpointNoFOC(double unitsPerSecond, int slot) {
    talon.SetControl(
        velocityVoltageControlNoFOC.WithVelocity(units::angular_velocity::turns_per_second_t{
            UnitsToRotor(unitsPerSecond)
        }).WithSlot(slot));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot) {
    talon.SetControl(motionMagicVelocityVoltageControl
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
        .WithFeedForward(0_V));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot, double feedforward) {
    talon.SetControl(motionMagicVelocityVoltageControl
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
        .WithFeedForward(units::voltage::volt_t{feedforward}));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVelocityMotionMagicSetpointNoFOC(double unitsPerSecond, int slot) {
    talon.SetControl(motionMagicVelocityVoltageControlNoFOC
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
        .WithFeedForward(0_V));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVelocityMotionMagicTorqueCurrentFOC(double unitsPerSecond, int slot) {
    talon.SetControl(motionMagicVelocityTorqueCurrentFOC
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
        .WithFeedForward(0_A));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVelocitySetpointIgnoreLimits(double unitsPerSecond, int slot) {
    talon.SetControl(
        velocityVoltageControlIgnoreLimits.WithVelocity(units::angular_velocity::turns_per_second_t{
            UnitsToRotor(unitsPerSecond)
        }).WithSlot(slot));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVelocitySetpointNoFOCIgnoreLimits(double unitsPerSecond, int slot) {
    talon.SetControl(
        velocityVoltageControlNoFOCIgnoreLimits.WithVelocity(units::angular_velocity::turns_per_second_t{
            UnitsToRotor(unitsPerSecond)
        }).WithSlot(slot));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVelocityMotionMagicSetpointIgnoreLimits(double unitsPerSecond, int slot) {
    talon.SetControl(motionMagicVelocityVoltageControlIgnoreLimits
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
        .WithFeedForward(0_V));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVelocityMotionMagicSetpointNoFOCIgnoreLimits(double unitsPerSecond, int slot) {
    talon.SetControl(motionMagicVelocityVoltageControlNoFOCIgnoreLimits
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
        .WithFeedForward(0_V));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVoltageOutput(double voltage) {
    talon.SetControl(voltageControl.WithOutput(units::voltage::volt_t{voltage}));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetCurrentPositionAsZero() {
    SetCurrentPosition(0);
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetCurrentPosition(double positionUnits) {
    talon.SetPosition(units::angle::turn_t{UnitsToRotor(positionUnits)});
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetEnableSoftLimits(bool forward, bool reverse) {
    config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    ctre_util::ApplyConfiguration(talon, config.fxConfig.SoftwareLimitSwitch);
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetEnableHardLimits(bool forward, bool reverse) {
    config.fxConfig.HardwareLimitSwitch.ForwardLimitEnable = forward;
    config.fxConfig.HardwareLimitSwitch.ReverseLimitEnable = reverse;
    ctre_util::ApplyConfiguration(talon, config.fxConfig.HardwareLimitSwitch);
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetEnableAutosetPositionValue(bool forward, bool reverse) {
    config.fxConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forward;
    config.fxConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverse;
    config.fxConfig.HardwareLimitSwitch.ForwardLimitEnable = forward;
    config.fxConfig.HardwareLimitSwitch.ReverseLimitEnable = reverse;

    ctre_util::ApplyConfiguration(talon, config.fxConfig.HardwareLimitSwitch);
}

template<typename pos_t>
void TalonFXIO<pos_t>::Follow(const CANDeviceId& masterId, bool opposeMasterDirection) {
    talon.SetControl(followerControl
        .WithLeaderID(masterId.GetDeviceNumber())
        .WithMotorAlignment(opposeMasterDirection
                                ? ctre::phoenix6::signals::MotorAlignmentValue::Opposed
                                : ctre::phoenix6::signals::MotorAlignmentValue::Aligned));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetTorqueCurrentFOC(double current) {
    talon.SetControl(torqueCurrentFOC.WithOutput(units::current::ampere_t{current}));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVelocityTorqueCurrentFOC(double unitsPerSecond, double feedforward) {
    talon.SetControl(velocityTorqueCurrentFOC
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithFeedForward(units::current::ampere_t{feedforward}));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetPositionTorqueCurrentFOC(double units, double feedforward) {
    talon.SetControl(positionTorqueCurrentFOC
        .WithPosition(units::angle::turn_t{ClampPosition(units)})
        .WithFeedForward(units::current::ampere_t{feedforward}));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetMotionMagicTorqueCurrentFOC(double units, int slot) {
    talon.SetControl(motionMagicTorqueCurrentFOC
        .WithPosition(units::angle::turn_t{ClampPosition(units)})
        .WithSlot(slot));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetMotionMagicTorqueCurrentFOC(double units, double velocity, double acceleration, double jerk,
                                                      int slot, double feedforward) {
    talon.SetControl(dynamicMotionMagicTorqueCurrentFOC
        .WithPosition(units::angle::turn_t{ClampPosition(units)})
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(velocity)})
        .WithAcceleration(units::angular_acceleration::turns_per_second_squared_t{UnitsToRotor(acceleration)})
        .WithJerk(units::angular_jerk::turns_per_second_cubed_t{UnitsToRotor(jerk)})
        .WithSlot(slot)
        .WithFeedForward(units::current::ampere_t{feedforward}));
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetMotionMagicConfig(const ctre::phoenix6::configs::MotionMagicConfigs& mmConfig) {
    config.fxConfig.MotionMagic = mmConfig;
    ctre_util::ApplyConfiguration(talon, config.fxConfig.MotionMagic);
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetVoltageConfig(const ctre::phoenix6::configs::VoltageConfigs& voltageConfig) {
    config.fxConfig.Voltage = voltageConfig;
    ctre_util::ApplyConfiguration(talon, config.fxConfig.Voltage);
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetSupplyCurrentLimit(double amps) {
    if (std::abs(lastAppliedSupplyLimitAmps - amps) < 1e-12) {
        return;
    }
    lastAppliedSupplyLimitAmps = amps;
    config.fxConfig.CurrentLimits.SupplyCurrentLimit = units::ampere_t{amps};
    config.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    ctre_util::ApplyConfigurationNonBlocking(talon, config.fxConfig.CurrentLimits);
}

template<typename pos_t>
void TalonFXIO<pos_t>::SetStatorCurrentLimit(double amps) {
    if (std::abs(lastAppliedStatorLimitAmps - amps) < 1e-12) {
        return;
    }
    lastAppliedStatorLimitAmps = amps;
    config.fxConfig.CurrentLimits.StatorCurrentLimit = units::ampere_t{amps};
    config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    ctre_util::ApplyConfigurationNonBlocking(talon, config.fxConfig.CurrentLimits);
}
