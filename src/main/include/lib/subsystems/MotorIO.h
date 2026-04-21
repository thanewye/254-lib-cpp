#pragma once

#include "lib/subsystems/MotorInputs.h"

class MotorIO {
public:
    virtual ~MotorIO() = default;

    virtual void UpdateInputs(MotorInputs& inputs) = 0;

    virtual void SetOpenLoopDutyCycle(double dutyCycle) = 0;
    virtual void SetOpenLoopDutyCycleNoFOC(double dutyCycle) = 0;
    virtual void SetOpenLoopDutyCycleIgnoreLimits(double dutyCycle) = 0;

    virtual void SetPositionSetpoint(double units) = 0;
    virtual void SetMotionMagicSetpoint(double units, int slot = 0) = 0;

    virtual void SetVelocitySetpoint(double unitsPerSecond, int slot = 0) = 0;
    virtual void SetVelocitySetpointNoFOC(double unitsPerSecond, int slot = 0) = 0;
    virtual void SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot = 0) = 0;
    virtual void SetVelocitySetpointIgnoreLimits(double unitsPerSecond, int slot = 0) = 0;

    virtual void SetVoltageOutput(double voltage) = 0;
    virtual void SetCurrentPositionAsZero() = 0;
    virtual void SetCurrentPosition(double positionUnits) = 0;

    virtual void SetEnableSoftLimits(bool forward, bool reverse) = 0;
    virtual void SetEnableHardLimits(bool forward, bool reverse) = 0;

    virtual void SetSupplyCurrentLimit(double amps) = 0;
    virtual void SetStatorCurrentLimit(double amps) = 0;
};
