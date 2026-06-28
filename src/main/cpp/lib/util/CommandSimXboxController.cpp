#include "lib/util/CommandSimXboxController.h"

#include "lib/util/ControllerMappings.h"

CommandSimXboxController::CommandSimXboxController(int port)
    : CommandXboxController(port)
    , hid(port, MappingForType(frc2026::constants::kSimControllerType))
    , mapping(&MappingForType(frc2026::constants::kSimControllerType)) {}

const SimXboxController* CommandSimXboxController::GetHID() const {
    return &hid;
}

frc2::Trigger CommandSimXboxController::A(frc::EventLoop* loop) const {
    return Button(mapping->GetButton("A"), loop);
}

frc2::Trigger CommandSimXboxController::B(frc::EventLoop* loop) const {
    return Button(mapping->GetButton("B"), loop);
}

frc2::Trigger CommandSimXboxController::X(frc::EventLoop* loop) const {
    return Button(mapping->GetButton("X"), loop);
}

frc2::Trigger CommandSimXboxController::Y(frc::EventLoop* loop) const {
    return Button(mapping->GetButton("Y"), loop);
}

frc2::Trigger CommandSimXboxController::LeftBumper(frc::EventLoop* loop) const {
    return Button(mapping->GetButton("LeftBumper"), loop);
}

frc2::Trigger CommandSimXboxController::RightBumper(frc::EventLoop* loop) const {
    return Button(mapping->GetButton("RightBumper"), loop);
}

frc2::Trigger CommandSimXboxController::Back(frc::EventLoop* loop) const {
    return Button(mapping->GetButton("Back"), loop);
}

frc2::Trigger CommandSimXboxController::Start(frc::EventLoop* loop) const {
    return Button(mapping->GetButton("Start"), loop);
}

frc2::Trigger CommandSimXboxController::LeftStick(frc::EventLoop* loop) const {
    return Button(mapping->GetButton("LeftStick"), loop);
}

frc2::Trigger CommandSimXboxController::RightStick(frc::EventLoop* loop) const {
    return Button(mapping->GetButton("RightStick"), loop);
}

frc2::Trigger CommandSimXboxController::LeftTrigger(double threshold, frc::EventLoop* loop) const {
    return AxisGreaterThan(mapping->GetAxis("LeftTrigger"), threshold, loop);
}

frc2::Trigger CommandSimXboxController::RightTrigger(double threshold, frc::EventLoop* loop) const {
    return AxisGreaterThan(mapping->GetAxis("RightTrigger"), threshold, loop);
}

double CommandSimXboxController::GetLeftX() const {
    return GetHID()->GetRawAxis(mapping->GetAxis("LeftX"));
}

double CommandSimXboxController::GetRightX() const {
    return GetHID()->GetRawAxis(mapping->GetAxis("RightX"));
}

double CommandSimXboxController::GetLeftY() const {
    return GetHID()->GetRawAxis(mapping->GetAxis("LeftY"));
}

double CommandSimXboxController::GetRightY() const {
    return GetHID()->GetRawAxis(mapping->GetAxis("RightY"));
}

const ControllerMapping& CommandSimXboxController::MappingForType(frc2026::constants::SimControllerType type) {
    switch (type) {
    case frc2026::constants::DUAL_SENSE:
        return controller_mappings::DUALSENSE_MAPPING;
    default:
        return controller_mappings::XBOX_MAPPING;
    }
}
