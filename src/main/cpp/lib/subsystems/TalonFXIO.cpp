#include "lib/subsystems/TalonFXIO.h"
#include "lib/util/CANStatusLogger.h"
#include "lib/util/CTREUtil.h"
#include "lib/util/StatusSignalManager.h"

TalonFXIO::TalonFXIO(const ServoMotorSubsystemConfig& config)
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
    CTREUtil::ConfigureTalonFX(talon, config.fxConfig);
    CTREUtil::TryUntilOk(
        [&] {
            return ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
                units::frequency::hertz_t{config.updateFrequencyHz}, signals);
        }, talon.GetDeviceID());
    CTREUtil::TryUntilOk(
        [&] { return talon.OptimizeBusUtilization(); }, talon.GetDeviceID());
    CANStatusLogger::GetInstance().RegisterTalonFX(config.name, &talon, config.talonCANID);
    for (auto* sig : signals) {
        StatusSignalManager::GetInstance().Register(sig);
    }
}

void TalonFXIO::ReadInputs(MotorInputs& inputs) {
    inputs.unitPosition = RotorToUnits(positionSignal.GetValueAsDouble());
    inputs.velocityUnitsPerSecond = RotorToUnits(velocitySignal.GetValueAsDouble());
    inputs.appliedVolts = voltageSignal.GetValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.GetValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.GetValueAsDouble();
    inputs.rawRotorPosition = rawRotorPositionSignal.GetValueAsDouble();
    inputs.motorTempCelsius = temperatureSignal.GetValueAsDouble();
}

void TalonFXIO::SetOpenLoopDutyCycle(double dutyCycle) {
    talon.SetControl(dutyCycleControl.WithOutput(dutyCycle));
}

void TalonFXIO::SetOpenLoopDutyCycleNoFOC(double dutyCycle) {
    talon.SetControl(dutyCycleControlNoFOC.WithOutput(dutyCycle));
}

void TalonFXIO::SetOpenLoopDutyCycleIgnoreLimits(double dutyCycle) {
    talon.SetControl(dutyCycleControlIgnoreLimits.WithOutput(dutyCycle));
}

void TalonFXIO::SetPositionSetpoint(double units) {
    talon.SetControl(positionVoltageControl.WithPosition(units::angle::turn_t{ClampPosition(units)}));
}

void TalonFXIO::SetMotionMagicSetpoint(double units, int slot) {
    talon.SetControl(
        motionMagicPositionControl.WithPosition(units::angle::turn_t{ClampPosition(units)}).WithSlot(slot));
}

void TalonFXIO::SetMotionMagicSetpoint(double units, double velocity, double acceleration, double jerk, int slot,
                                       double feedforward) {
    talon.SetControl(dynamicMotionMagicVoltage
        .WithPosition(units::angle::turn_t{ClampPosition(units)})
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(velocity)})
        .WithAcceleration(units::angular_acceleration::turns_per_second_squared_t{UnitsToRotor(acceleration)})
        .WithJerk(units::angular_jerk::turns_per_second_cubed_t{UnitsToRotor(jerk)})
        .WithSlot(slot)
        .WithFeedForward(units::voltage::volt_t{feedforward}));
}

void TalonFXIO::SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode) {
    config.fxConfig.MotorOutput.NeutralMode = mode;
    talon.SetNeutralMode(mode);
}

void TalonFXIO::SetVelocitySetpoint(double unitsPerSecond, int slot) {
    talon.SetControl(
        velocityVoltageControl.WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)}).
        WithSlot(slot));
}

void TalonFXIO::SetVelocitySetpointNoFOC(double unitsPerSecond, int slot) {
    talon.SetControl(
        velocityVoltageControlNoFOC.WithVelocity(units::angular_velocity::turns_per_second_t{
            UnitsToRotor(unitsPerSecond)
        }).WithSlot(slot));
}

void TalonFXIO::SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot) {
    talon.SetControl(motionMagicVelocityVoltageControl
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
}

void TalonFXIO::SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot, double feedforward) {
    talon.SetControl(motionMagicVelocityVoltageControl
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
        .WithFeedForward(units::voltage::volt_t{feedforward}));
}

void TalonFXIO::SetVelocityMotionMagicSetpointNoFOC(double unitsPerSecond, int slot) {
    talon.SetControl(motionMagicVelocityVoltageControlNoFOC
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
}

void TalonFXIO::SetVelocityMotionMagicTorqueCurrentFOC(double unitsPerSecond, int slot) {
    talon.SetControl(motionMagicVelocityTorqueCurrentFOC
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
}

void TalonFXIO::SetVelocitySetpointIgnoreLimits(double unitsPerSecond, int slot) {
    talon.SetControl(
        velocityVoltageControlIgnoreLimits.WithVelocity(units::angular_velocity::turns_per_second_t{
            UnitsToRotor(unitsPerSecond)
        }).WithSlot(slot));
}

void TalonFXIO::SetVelocitySetpointNoFOCIgnoreLimits(double unitsPerSecond, int slot) {
    talon.SetControl(
        velocityVoltageControlNoFOCIgnoreLimits.WithVelocity(units::angular_velocity::turns_per_second_t{
            UnitsToRotor(unitsPerSecond)
        }).WithSlot(slot));
}

void TalonFXIO::SetVelocityMotionMagicSetpointIgnoreLimits(double unitsPerSecond, int slot) {
    talon.SetControl(motionMagicVelocityVoltageControlIgnoreLimits
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
}

void TalonFXIO::SetVelocityMotionMagicSetpointNoFOCIgnoreLimits(double unitsPerSecond, int slot) {
    talon.SetControl(motionMagicVelocityVoltageControlNoFOCIgnoreLimits
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithAcceleration(config.fxConfig.MotionMagic.MotionMagicAcceleration)
        .WithSlot(slot)
}

void TalonFXIO::SetVoltageOutput(double voltage) {
    talon.SetControl(voltageControl.WithOutput(units::voltage::volt_t{voltage}));
}

void TalonFXIO::SetCurrentPositionAsZero() {
    SetCurrentPosition(0);
}

void TalonFXIO::SetCurrentPosition(double positionUnits) {
    talon.SetPosition(units::angle::turn_t{UnitsToRotor(positionUnits)});
}

void TalonFXIO::SetEnableSoftLimits(bool forward, bool reverse) {
    config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = forward;
    config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverse;
    CTREUtil::ApplyConfiguration(talon, config.fxConfig.SoftwareLimitSwitch);
}

void TalonFXIO::SetEnableHardLimits(bool forward, bool reverse) {
    config.fxConfig.HardwareLimitSwitch.ForwardLimitEnable = forward;
    config.fxConfig.HardwareLimitSwitch.ReverseLimitEnable = reverse;
    CTREUtil::ApplyConfiguration(talon, config.fxConfig.HardwareLimitSwitch);
}

void TalonFXIO::SetEnableAutosetPositionValue(bool forward, bool reverse) {
    config.fxConfig.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forward;
    config.fxConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverse;
    config.fxConfig.HardwareLimitSwitch.ForwardLimitEnable = forward;
    config.fxConfig.HardwareLimitSwitch.ReverseLimitEnable = reverse;

    CTREUtil::ApplyConfiguration(talon, config.fxConfig.HardwareLimitSwitch);
}

void TalonFXIO::Follow(const CANDeviceId& masterId, bool opposeMasterDirection) {
    talon.SetControl(followerControl
        .WithLeaderID(masterId.GetDeviceNumber())
        .WithMotorAlignment(opposeMasterDirection
                                ? ctre::phoenix6::signals::MotorAlignmentValue::Opposed
                                : ctre::phoenix6::signals::MotorAlignmentValue::Aligned));
}

void TalonFXIO::SetTorqueCurrentFOC(double current) {
    talon.SetControl(torqueCurrentFOC.WithOutput(units::current::ampere_t{current}));
}

void TalonFXIO::SetVelocityTorqueCurrentFOC(double unitsPerSecond, double feedforward) {
    talon.SetControl(velocityTorqueCurrentFOC
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(unitsPerSecond)})
        .WithFeedForward(units::current::ampere_t{feedforward}));
}

void TalonFXIO::SetPositionTorqueCurrentFOC(double units, double feedforward) {
    talon.SetControl(positionTorqueCurrentFOC
        .WithPosition(units::angle::turn_t{ClampPosition(units)})
        .WithFeedForward(units::current::ampere_t{feedforward}));
}

void TalonFXIO::SetMotionMagicTorqueCurrentFOC(double units, int slot) {
    talon.SetControl(motionMagicTorqueCurrentFOC
        .WithPosition(units::angle::turn_t{ClampPosition(units)})
        .WithSlot(slot));
}

void TalonFXIO::SetMotionMagicTorqueCurrentFOC(double units, double velocity, double acceleration, double jerk,
                                               int slot, double feedforward) {
    talon.SetControl(dynamicMotionMagicTorqueCurrentFOC
        .WithPosition(units::angle::turn_t{ClampPosition(units)})
        .WithVelocity(units::angular_velocity::turns_per_second_t{UnitsToRotor(velocity)})
        .WithAcceleration(units::angular_acceleration::turns_per_second_squared_t{UnitsToRotor(acceleration)})
        .WithJerk(units::angular_jerk::turns_per_second_cubed_t{UnitsToRotor(jerk)})
        .WithSlot(slot)
        .WithFeedForward(units::current::ampere_t{feedforward}));
}

void TalonFXIO::SetMotionMagicConfig(const ctre::phoenix6::configs::MotionMagicConfigs& mmConfig) {
    config.fxConfig.MotionMagic = mmConfig;
    CTREUtil::ApplyConfiguration(talon, config.fxConfig.MotionMagic);
}

void TalonFXIO::SetVoltageConfig(const ctre::phoenix6::configs::VoltageConfigs& voltageConfig) {
    config.fxConfig.Voltage = voltageConfig;
    CTREUtil::ApplyConfiguration(talon, config.fxConfig.Voltage);
}

void TalonFXIO::SetSupplyCurrentLimit(double amps) {
    if (std::abs(lastAppliedSupplyLimitAmps - amps) < 1e-12) {
        return;
    }
    lastAppliedSupplyLimitAmps = amps;
    config.fxConfig.CurrentLimits.SupplyCurrentLimit = units::ampere_t{amps};
    config.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    CTREUtil::ApplyConfigurationNonBlocking(talon, config.fxConfig.CurrentLimits);
}

void TalonFXIO::SetStatorCurrentLimit(double amps) {
    if (std::abs(lastAppliedStatorLimitAmps - amps) < 1e-12) {
        return;
    }
    lastAppliedStatorLimitAmps = amps;
    config.fxConfig.CurrentLimits.StatorCurrentLimit = units::ampere_t{amps};
    config.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    CTREUtil::ApplyConfigurationNonBlocking(talon, config.fxConfig.CurrentLimits);
}
