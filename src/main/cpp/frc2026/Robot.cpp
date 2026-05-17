// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc2/command/CommandScheduler.h>

#include "akit/Logger.h"
#include "akit/NT4Publisher.h"
#include "akit/WPILOGWriter.h"
#include "frc2026/Robot.h"

#include <frc/geometry/Pose2d.h>

namespace {
    akit::networktables::NT4Publisher& GetNT4Publisher() {
        static akit::networktables::NT4Publisher publisher;
        return publisher;
    }

    akit::wpilog::WPILOGWriter& GetWPILOGWriter() {
        static akit::wpilog::WPILOGWriter wpilog;
        return wpilog;
    }
} // namespace

Robot::Robot() {
    frc::DataLogManager::Start();
    frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
    akit::Logger::AddDataReceiver(&GetNT4Publisher());
    akit::Logger::AddDataReceiver(&GetWPILOGWriter());
    akit::Logger::Start();
}

void Robot::RobotPeriodic() {
    akit::Logger::PeriodicBeforeUser();
    akit::Logger::RecordOutput("stuff", frc::Pose2d(1.0_m, 26.0_in, 5_rad));
    frc2::CommandScheduler::GetInstance().Run();
    akit::Logger::PeriodicAfterUser();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.value());
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        m_autonomousCommand->Cancel();
    }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
