#pragma once

#include "lib/subsystems/MotorIO.h"

/*
 * no-op motor IO for use in advantagekit replay (when c++ version comes out por favor)
 */
class ReplayMotorIO : public MotorIO {
public:
    void ReadInputs(MotorInputs& inputs) override {}

    void SetOpenLoopDutyCycle(double dutyCycle) override {}
    void SetOpenLoopDutyCycleNoFOC(double dutyCycle) override {}
    void SetOpenLoopDutyCycleIgnoreLimits(double dutyCycle) override {}

    void SetPositionSetpoint(double units) override {}
    void SetMotionMagicSetpoint(double units, int slot) override {}

    void SetMotionMagicSetpoint(double units, double velocity, double acceleration, double jerk, int slot, double feedforward) override {}

    void SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode) override {}

    void SetVelocitySetpoint(double unitsPerSecond, int slot) override {}
    void SetVelocitySetpointNoFOC(double unitsPerSecond, int slot) override {}
    void SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot) override {}
    void SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot, double feedforward) override {}
    void SetVelocityMotionMagicSetpointNoFOC(double unitsPerSecond, int slot) override {}
    void SetVelocityMotionMagicTorqueCurrentFOC(double unitsPerSecond, int slot) override {}
    void SetVelocitySetpointIgnoreLimits(double unitsPerSecond, int slot) override {}
    void SetVelocitySetpointNoFOCIgnoreLimits(double unitsPerSecond, int slot) override {}
    void SetVelocityMotionMagicSetpointIgnoreLimits(double unitsPerSecond, int slot) override {}
    void SetVelocityMotionMagicSetpointNoFOCIgnoreLimits(double unitsPerSecond, int slot) override {}

    void SetVoltageOutput(double voltage) override {}

    void SetCurrentPositionAsZero() override {}
    void SetCurrentPosition(double positionUnits) override {}

    void SetEnableSoftLimits(bool forward, bool reverse) override {}
    void SetEnableHardLimits(bool forward, bool reverse) override {}
    void SetEnableAutosetPositionValue(bool forward, bool reverse) override {}

    void Follow(const CANDeviceId& masterId, bool opposeMasterDirection) override {}

    void SetTorqueCurrentFOC(double current) override {}
    void SetVelocityTorqueCurrentFOC(double unitsPerSecond, double feedforward) override {}
    void SetPositionTorqueCurrentFOC(double units, double feedforward) override {}
    void SetMotionMagicTorqueCurrentFOC(double units, int slot) override {}

    void SetMotionMagicTorqueCurrentFOC(double units, double velocity, double acceleration, double jerk, int slot, double feedforward) override {}

    void SetMotionMagicConfig(const ctre::phoenix6::configs::MotionMagicConfigs& config) override {}
    void SetVoltageConfig(const ctre::phoenix6::configs::VoltageConfigs& config) override {}
    void SetSupplyCurrentLimit(double amps) override {}
    void SetStatorCurrentLimit(double amps) override {}
};
