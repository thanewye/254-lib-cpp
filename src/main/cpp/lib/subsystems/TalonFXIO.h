#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/DynamicMotionMagicTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/DynamicMotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/MotionMagicTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/MotionMagicVelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/MotionMagicVelocityVoltage.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/TorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VelocityTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include "lib/subsystems/MotorIO.h"
#include "lib/subsystems/ServoMotorSubsystemConfig.h"
#include "lib/util/CTREUtil.h"
#include "lib/util/CANStatusLogger.h"

class TalonFXIO : public MotorIO {
protected:
    ctre::phoenix6::hardware::TalonFX talon;
    ServoMotorSubsystemConfig config;

    ctre::phoenix6::controls::DutyCycleOut dutyCycleControl = 
            ctre::phoenix6::controls::DutyCycleOut(0).WithEnableFOC(true);
    ctre::phoenix6::controls::DutyCycleOut dutyCycleControlIgnoreLimits = 
            ctre::phoenix6::controls::DutyCycleOut(0).WithEnableFOC(true).WithIgnoreHardwareLimits(true);
    ctre::phoenix6::controls::DutyCycleOut dutyCycleControlNoFOC = 
            ctre::phoenix6::controls::DutyCycleOut(0).WithEnableFOC(false);

    ctre::phoenix6::controls::VelocityVoltage velocityVoltageControl = 
            ctre::phoenix6::controls::VelocityVoltage(0_rad_per_s).WithEnableFOC(true);
    ctre::phoenix6::controls::VelocityVoltage velocityVoltageControlNoFOC = 
            ctre::phoenix6::controls::VelocityVoltage(0_rad_per_s).WithEnableFOC(false);
    ctre::phoenix6::controls::VelocityVoltage velocityVoltageControlIgnoreLimits = 
            ctre::phoenix6::controls::VelocityVoltage(0_rad_per_s).WithEnableFOC(true).WithIgnoreHardwareLimits(true);
    ctre::phoenix6::controls::VelocityVoltage velocityVoltageControlNoFOCIgnoreLimits = 
            ctre::phoenix6::controls::VelocityVoltage(0_rad_per_s).WithEnableFOC(false).WithIgnoreHardwareLimits(true);

    ctre::phoenix6::controls::MotionMagicVelocityVoltage motionMagicVelocityVoltageControl = 
            ctre::phoenix6::controls::MotionMagicVelocityVoltage(0_rad_per_s);
    ctre::phoenix6::controls::MotionMagicVelocityVoltage motionMagicVelocityVoltageControlNoFOC = 
            ctre::phoenix6::controls::MotionMagicVelocityVoltage(0_rad_per_s).WithEnableFOC(false);
    ctre::phoenix6::controls::MotionMagicVelocityVoltage motionMagicVelocityVoltageControlIgnoreLimits = 
            ctre::phoenix6::controls::MotionMagicVelocityVoltage(0_rad_per_s).WithEnableFOC(true).WithIgnoreHardwareLimits(true);
    ctre::phoenix6::controls::MotionMagicVelocityVoltage motionMagicVelocityVoltageControlNoFOCIgnoreLimits = 
            ctre::phoenix6::controls::MotionMagicVelocityVoltage(0_rad_per_s).WithEnableFOC(false).WithIgnoreHardwareLimits(true);

    ctre::phoenix6::controls::VoltageOut voltageControl = 
            ctre::phoenix6::controls::VoltageOut(0_V);

    ctre::phoenix6::controls::PositionVoltage positionVoltageControl = 
            ctre::phoenix6::controls::PositionVoltage(0_rad);

    ctre::phoenix6::controls::MotionMagicVoltage motionMagicPositionControl = 
            ctre::phoenix6::controls::MotionMagicVoltage(0_rad);

    ctre::phoenix6::controls::DynamicMotionMagicVoltage dynamicMotionMagicVoltage = 
            ctre::phoenix6::controls::DynamicMotionMagicVoltage(0_rad, 0_rad_per_s, 0_rad_per_s_sq);

    ctre::phoenix6::controls::Follower followerControl = 
            ctre::phoenix6::controls::Follower(0, ctre::phoenix6::signals::MotorAlignmentValue::Aligned);

    ctre::phoenix6::controls::TorqueCurrentFOC torqueCurrentFOC = 
            ctre::phoenix6::controls::TorqueCurrentFOC(0_A);

    ctre::phoenix6::controls::VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = 
            ctre::phoenix6::controls::VelocityTorqueCurrentFOC(0_rad_per_s);

    ctre::phoenix6::controls::PositionTorqueCurrentFOC positionTorqueCurrentFOC = 
            ctre::phoenix6::controls::PositionTorqueCurrentFOC(0_rad);

    ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOC = 
            ctre::phoenix6::controls::MotionMagicTorqueCurrentFOC(0_rad);

    ctre::phoenix6::controls::DynamicMotionMagicTorqueCurrentFOC dynamicMotionMagicTorqueCurrentFOC = 
            ctre::phoenix6::controls::DynamicMotionMagicTorqueCurrentFOC(0_rad, 0_rad_per_s, 0_rad_per_s_sq);

    ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC motionMagicVelocityTorqueCurrentFOC = 
            ctre::phoenix6::controls::MotionMagicVelocityTorqueCurrentFOC(0_rad_per_s);

    ctre::phoenix6::StatusSignal<units::turn_t> positionSignal;
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> velocitySignal;
    ctre::phoenix6::StatusSignal<units::volt_t> voltageSignal;
    ctre::phoenix6::StatusSignal<units::ampere_t> currentStatorSignal;
    ctre::phoenix6::StatusSignal<units::ampere_t> currentSupplySignal;
    ctre::phoenix6::StatusSignal<units::turn_t> rawRotorPositionSignal;
    ctre::phoenix6::StatusSignal<units::celsius_t> temperatureSignal;

    ctre::phoenix6::BaseStatusSignal* signals[7];

public:
    TalonFXIO(const ServoMotorSubsystemConfig& config)
        : talon(config.talonCANID.GetDeviceNumber(), config.talonCANID.GetBus())
        , config(config)
        , positionSignal(talon.GetPosition())
        , velocitySignal(talon.GetVelocity())
        , voltageSignal(talon.GetMotorVoltage())
        , currentStatorSignal(talon.GetStatorCurrent())
        , currentSupplySignal(talon.GetSupplyCurrent())
        , rawRotorPositionSignal(talon.GetRotorPosition())
        , temperatureSignal(talon.GetDeviceTemp())
        , signals{&positionSignal, &velocitySignal, &voltageSignal, &currentStatorSignal, &currentSupplySignal, &rawRotorPositionSignal, &temperatureSignal}
        {
            CTREUtil::ConfigureTalonFX(talon, config.fxConfig);
            CTREUtil::TryUntilOk(
                [&] { return ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
                    units::frequency::hertz_t{config.updateFrequencyHz}, signals); }, talon.GetDeviceID());
            CTREUtil::TryUntilOk(
                [&] { return talon.OptimizeBusUtilization(); }, talon.GetDeviceID());
            CANStatusLogger::GetInstance().RegisterTalonFX(config.name, &talon, config.talonCANID);
            // status signal manager register signals
        };

    void UpdateInputs(MotorInputs& inputs) override {}

    void SetOpenLoopDutyCycle(double dutyCycle) override {}
    void SetOpenLoopDutyCycleNoFOC(double dutyCycle) override {}
    void SetOpenLoopDutyCycleIgnoreLimits(double dutyCycle) override {}

    void SetPositionSetpoint(double units) override {}
    void SetMotionMagicSetpoint(double units, int slot = 0) override {}

    void SetVelocitySetpoint(double unitsPerSecond, int slot = 0) override {}
    void SetVelocitySetpointNoFOC(double unitsPerSecond, int slot = 0) override {}
    void SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot = 0) override {}
    void SetVelocitySetpointIgnoreLimits(double unitsPerSecond, int slot = 0) override {}

    void SetVoltageOutput(double voltage) override {}
    void SetCurrentPositionAsZero() override {}
    void SetCurrentPosition(double positionUnits) override {}

    void SetEnableSoftLimits(bool forward, bool reverse) override {}
    void SetEnableHardLimits(bool forward, bool reverse) override {}

    void SetSupplyCurrentLimit(double amps) override {}
    void SetStatorCurrentLimit(double amps) override {}
};
