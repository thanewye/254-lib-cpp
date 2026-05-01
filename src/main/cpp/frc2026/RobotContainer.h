// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "lib/subsystems/MotorIO.h"

class RobotContainer {
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    void ConfigureBindings();

    std::unique_ptr<MotorIO> m_compileProofServoIO;
    std::unique_ptr<frc2::SubsystemBase> m_compileProofServoSubsystem;
};
