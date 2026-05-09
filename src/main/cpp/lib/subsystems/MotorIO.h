#pragma once

#include <ctre/phoenix6/configs/MotionMagicConfigs.hpp>
#include <ctre/phoenix6/configs/VoltageConfigs.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#include "lib/drivers/CANDeviceId.h"
#include "lib/subsystems/MotorInputs.h"

class MotorIO {
public:
    virtual ~MotorIO() = default;

    virtual void ReadInputs(MotorInputs& inputs) = 0;

    virtual void SetOpenLoopDutyCycle(double dutyCycle) = 0;
    virtual void SetOpenLoopDutyCycleNoFOC(double dutyCycle) = 0;

    /**
     * Same as SetOpenLoopDutyCycle but request ignores hardware limit switches (e.g. feeder shoot-through).
     */
    virtual void SetOpenLoopDutyCycleIgnoreLimits(double dutyCycle) = 0;

    // These are in the "units" of the subsystem (rad, m).
    virtual void SetPositionSetpoint(double units) = 0;

    virtual void SetMotionMagicSetpoint(double units, int slot = 0) = 0;
    virtual void SetMotionMagicSetpoint(double units, double velocity, double acceleration, double jerk, int slot = 0, double feedforward = 0.0) = 0;

    virtual void SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode) = 0;

    virtual void SetVelocitySetpoint(double unitsPerSecond, int slot = 0) = 0;
    virtual void SetVelocitySetpointNoFOC(double unitsPerSecond, int slot = 0) = 0;

    virtual void SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot = 0) = 0;
    virtual void SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot, double feedforward) = 0;
    virtual void SetVelocityMotionMagicSetpointNoFOC(double unitsPerSecond, int slot = 0) = 0;

    virtual void SetVelocityMotionMagicTorqueCurrentFOC(double unitsPerSecond, int slot = 0) = 0;

    virtual void SetVelocitySetpointIgnoreLimits(double unitsPerSecond, int slot = 0) = 0;
    virtual void SetVelocitySetpointNoFOCIgnoreLimits(double unitsPerSecond, int slot = 0) = 0;

    virtual void SetVelocityMotionMagicSetpointIgnoreLimits(double unitsPerSecond, int slot = 0) = 0;
    virtual void SetVelocityMotionMagicSetpointNoFOCIgnoreLimits(double unitsPerSecond, int slot = 0) = 0;

    virtual void SetVoltageOutput(double voltage) = 0;

    virtual void SetCurrentPositionAsZero() = 0;
    virtual void SetCurrentPosition(double positionUnits) = 0;

    virtual void SetEnableSoftLimits(bool forward, bool reverse) = 0;
    virtual void SetEnableHardLimits(bool forward, bool reverse) = 0;
    virtual void SetEnableAutosetPositionValue(bool forward, bool reverse) = 0;

    virtual void Follow(const CANDeviceId& masterId, bool opposeMasterDirection) = 0;

    virtual void SetTorqueCurrentFOC(double current) = 0;

    virtual void SetVelocityTorqueCurrentFOC(double unitsPerSecond, double feedforward = 0.0) = 0;

    virtual void SetPositionTorqueCurrentFOC(double units, double feedforward = 0.0) = 0;

    virtual void SetMotionMagicTorqueCurrentFOC(double units, int slot = 0) = 0;
    virtual void SetMotionMagicTorqueCurrentFOC(double units, double velocity, double acceleration, double jerk, int slot = 0, double feedforward = 0.0) = 0;

    virtual void SetMotionMagicConfig(const ctre::phoenix6::configs::MotionMagicConfigs& config) = 0;
    virtual void SetVoltageConfig(const ctre::phoenix6::configs::VoltageConfigs& config) = 0;

    virtual void SetSupplyCurrentLimit(double amps) = 0;
    virtual void SetStatorCurrentLimit(double amps) = 0;
};
