// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>

#include "frc2026/RobotContainer.h"
#include "lib/subsystems/MotorIO.h"
#include "lib/subsystems/ServoMotorSubsystem.h"

namespace {
    struct RobotContainerTestMotorInputs : MotorInputs {};

    class RobotContainerTestMotorIO : public MotorIO {
    public:
        void ReadInputs(MotorInputs &inputs) override {
            inputs.unitPosition = 0.0;
            inputs.velocityUnitsPerSecond = 0.0;
        }

        void SetOpenLoopDutyCycle(double dutyCycle) override {}

        void SetOpenLoopDutyCycleNoFOC(double dutyCycle) override {}

        void SetOpenLoopDutyCycleIgnoreLimits(double dutyCycle) override {}

        void SetPositionSetpoint(double units) override {}

        void SetMotionMagicSetpoint(double units, int slot) override {}

        void SetMotionMagicSetpoint(
            double units,
            double velocity,
            double acceleration,
            double jerk,
            int slot,
            double feedforward) override {}

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

        void Follow(const CANDeviceId &masterId, bool opposeMasterDirection) override {}

        void SetTorqueCurrentFOC(double current) override {}

        void SetVelocityTorqueCurrentFOC(double unitsPerSecond, double feedforward) override {}

        void SetPositionTorqueCurrentFOC(double units, double feedforward) override {}

        void SetMotionMagicTorqueCurrentFOC(double units, int slot) override {}

        void SetMotionMagicTorqueCurrentFOC(
            double units,
            double velocity,
            double acceleration,
            double jerk,
            int slot,
            double feedforward) override {}

        void SetMotionMagicConfig(const ctre::phoenix6::configs::MotionMagicConfigs &config) override {}

        void SetVoltageConfig(const ctre::phoenix6::configs::VoltageConfigs &config) override {}

        void SetSupplyCurrentLimit(double amps) override {}

        void SetStatorCurrentLimit(double amps) override {}
    };

    ServoMotorSubsystemConfig MakeCompileProofServoConfig() {
        return {
            .name = "Compile Proof Servo",
            .talonCANID = CANDeviceId(1),
        };
    }
} // namespace

RobotContainer::RobotContainer() {
    m_compileProofServoIO = std::make_unique<RobotContainerTestMotorIO>();
    m_compileProofServoSubsystem =
            std::make_unique<ServoMotorSubsystem<RobotContainerTestMotorInputs, RobotContainerTestMotorIO> >(
                MakeCompileProofServoConfig(),
                RobotContainerTestMotorInputs{},
                static_cast<RobotContainerTestMotorIO *>(m_compileProofServoIO.get()));
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return frc2::cmd::Print("No autonomous command configured");
}
