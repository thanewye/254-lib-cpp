// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc2/command/CommandPtr.h>

#include "akit/LoggedRobot.h"
#include "akit/autolog/AutoLogOutput.h"
#include "frc2026/RobotContainer.h"

class Robot : public akit::LoggedRobot {
public:
    Robot();

    void RobotPeriodic() override;

    void DisabledInit() override;

    void DisabledPeriodic() override;

    void DisabledExit() override;

    void AutonomousInit() override;

    void AutonomousPeriodic() override;

    void AutonomousExit() override;

    void TeleopInit() override;

    void TeleopPeriodic() override;

    void TeleopExit() override;

    void TestInit() override;

    void TestPeriodic() override;

    void TestExit() override;

private:
    std::optional<frc2::CommandPtr> m_autonomousCommand;

    RobotContainer m_container;
    AUTOLOG_OUTPUT(int, number);
};
