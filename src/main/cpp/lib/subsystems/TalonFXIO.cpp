#include "lib/subsystems/TalonFXIO.h"

// TODO: add Phoenix6 includes and implement all methods
// Reference: src/main/java/com/team254/lib/TalonFXIO.java

TalonFXIO::TalonFXIO(const ServoMotorSubsystemConfig& cfg)
    : talon(cfg.talonCANID.GetDeviceNumber(), cfg.talonCANID.GetBus()),
      config(cfg) {}

void TalonFXIO::UpdateInputs(MotorInputs& inputs) {}
void TalonFXIO::SetOpenLoopDutyCycle(double dutyCycle) {}
void TalonFXIO::SetOpenLoopDutyCycleNoFOC(double dutyCycle) {}
void TalonFXIO::SetOpenLoopDutyCycleIgnoreLimits(double dutyCycle) {}
void TalonFXIO::SetPositionSetpoint(double units) {}
void TalonFXIO::SetMotionMagicSetpoint(double units, int slot) {}
void TalonFXIO::SetVelocitySetpoint(double unitsPerSecond, int slot) {}
void TalonFXIO::SetVelocitySetpointNoFOC(double unitsPerSecond, int slot) {}
void TalonFXIO::SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot) {}
void TalonFXIO::SetVelocitySetpointIgnoreLimits(double unitsPerSecond, int slot) {}
void TalonFXIO::SetVoltageOutput(double voltage) {}
void TalonFXIO::SetCurrentPositionAsZero() {}
void TalonFXIO::SetCurrentPosition(double positionUnits) {}
void TalonFXIO::SetEnableSoftLimits(bool forward, bool reverse) {}
void TalonFXIO::SetEnableHardLimits(bool forward, bool reverse) {}
void TalonFXIO::SetSupplyCurrentLimit(double amps) {}
void TalonFXIO::SetStatorCurrentLimit(double amps) {}
