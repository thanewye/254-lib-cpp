#pragma once

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc/XboxController.h>

#include "SimXboxController.h"
#include "frc2026/Constants.h"
#include "lib/util/ControllerMapping.h"

class CommandSimXboxController : public frc2::CommandXboxController {
public:
    explicit CommandSimXboxController(int port);

    const SimXboxController* GetHID() const;

    frc2::Trigger A(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
    frc2::Trigger B(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
    frc2::Trigger X(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
    frc2::Trigger Y(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
    frc2::Trigger LeftBumper(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
    frc2::Trigger RightBumper(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
    frc2::Trigger Back(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
    frc2::Trigger Start(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
    frc2::Trigger LeftStick(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
    frc2::Trigger RightStick(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
    frc2::Trigger LeftTrigger(double threshold = 0.5, frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
    frc2::Trigger RightTrigger(double threshold = 0.5, frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;

    double GetLeftX() const;
    double GetRightX() const;
    double GetLeftY() const;
    double GetRightY() const;

private:
    SimXboxController hid;
    const ControllerMapping* mapping = nullptr;

    static const ControllerMapping& MappingForType(frc2026::constants::SimControllerType type);
};
