#pragma once

#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc/XboxController.h>

#include "ControllerMappings.h"
#include "SimXboxController.h"
#include "frc2026/Constants.h"
#include "lib/util/ControllerMapping.h"

class CommandSimXboxController : public frc2::CommandXboxController {
private:
    SimXboxController hid;
    const ControllerMapping* mapping = nullptr;

    static const ControllerMapping& MappingForType(frc2026::Constants::SimControllerType type) {
        switch (type) {
            case frc2026::Constants::DUAL_SENSE:
                return ControllerMappings::DUALSENSE_MAPPING;
            default:
                return ControllerMappings::XBOX_MAPPING;
        }
    }

public:
    explicit CommandSimXboxController(int port)
        : CommandXboxController(port),
          hid(port, MappingForType(frc2026::Constants::kSimControllerType)),
          mapping(&MappingForType(frc2026::Constants::kSimControllerType)) {}

    const SimXboxController* GetHID() const { return &hid; }

    frc2::Trigger A(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return Button(mapping->GetButton("A"), loop);
    }

    frc2::Trigger B(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return Button(mapping->GetButton("B"), loop);
    }

    frc2::Trigger X(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return Button(mapping->GetButton("X"), loop);
    }

    frc2::Trigger Y(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return Button(mapping->GetButton("Y"), loop);
    }

    frc2::Trigger LeftBumper(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return Button(mapping->GetButton("LeftBumper"), loop);
    }

    frc2::Trigger RightBumper(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return Button(mapping->GetButton("RightBumper"), loop);
    }

    frc2::Trigger Back(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return Button(mapping->GetButton("Back"), loop);
    }

    frc2::Trigger Start(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return Button(mapping->GetButton("Start"), loop);
    }

    frc2::Trigger LeftStick(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return Button(mapping->GetButton("LeftStick"), loop);
    }

    frc2::Trigger RightStick(frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return Button(mapping->GetButton("RightStick"), loop);
    }

    frc2::Trigger LeftTrigger(double threshold = 0.5, frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return AxisGreaterThan(mapping->GetAxis("LeftTrigger"), threshold, loop);
    }

    frc2::Trigger RightTrigger(double threshold = 0.5, frc::EventLoop* loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
        return AxisGreaterThan(mapping->GetAxis("RightTrigger"), threshold, loop);
    }

    double GetLeftX() const {
        return GetHID()->GetRawAxis(mapping->GetAxis("LeftX"));
    }

    double GetRightX() const {
        return GetHID()->GetRawAxis(mapping->GetAxis("RightX"));
    }

    double GetLeftY() const {
        return GetHID()->GetRawAxis(mapping->GetAxis("LeftY"));
    }

    double GetRightY() const {
        return GetHID()->GetRawAxis(mapping->GetAxis("RightY"));
    }
};
