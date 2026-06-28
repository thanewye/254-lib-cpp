#include "lib/util/SimXboxController.h"

#include <frc/event/BooleanEvent.h>

SimXboxController::SimXboxController(int port, const ControllerMapping& mapping)
    : frc::XboxController(port)
    , m_mapping(mapping) {}

double SimXboxController::GetLeftX() const {
    return GetRawAxis(m_mapping.GetAxis("LeftX"));
}
double SimXboxController::GetRightX() const {
    return GetRawAxis(m_mapping.GetAxis("RightX"));
}
double SimXboxController::GetLeftY() const {
    return GetRawAxis(m_mapping.GetAxis("LeftY"));
}
double SimXboxController::GetRightY() const {
    return GetRawAxis(m_mapping.GetAxis("RightY"));
}

double SimXboxController::GetLeftTriggerAxis() const {
    return GetRawAxis(m_mapping.GetAxis("LeftTrigger"));
}

frc::BooleanEvent SimXboxController::LeftTrigger(double threshold, frc::EventLoop* loop) const {
    return AxisGreaterThan(m_mapping.GetAxis("LeftTrigger"), threshold, loop);
}

frc::BooleanEvent SimXboxController::LeftTrigger(frc::EventLoop* loop) const {
    return LeftTrigger(0.5, loop);
}

double SimXboxController::GetRightTriggerAxis() const {
    return GetRawAxis(m_mapping.GetAxis("RightTrigger"));
}

frc::BooleanEvent SimXboxController::RightTrigger(double threshold, frc::EventLoop* loop) const {
    return AxisGreaterThan(m_mapping.GetAxis("RightTrigger"), threshold, loop);
}

frc::BooleanEvent SimXboxController::RightTrigger(frc::EventLoop* loop) const {
    return RightTrigger(0.5, loop);
}

bool SimXboxController::GetAButton() const {
    return GetRawButton(m_mapping.GetButton("A"));
}
bool SimXboxController::GetAButtonPressed() {
    return GetRawButtonPressed(m_mapping.GetButton("A"));
}
bool SimXboxController::GetAButtonReleased() {
    return GetRawButtonReleased(m_mapping.GetButton("A"));
}

frc::BooleanEvent SimXboxController::A(frc::EventLoop* loop) const {
    return frc::GenericHID::Button(m_mapping.GetButton("A"), loop);
}

bool SimXboxController::GetBButton() const {
    return GetRawButton(m_mapping.GetButton("B"));
}
bool SimXboxController::GetBButtonPressed() {
    return GetRawButtonPressed(m_mapping.GetButton("B"));
}
bool SimXboxController::GetBButtonReleased() {
    return GetRawButtonReleased(m_mapping.GetButton("B"));
}

frc::BooleanEvent SimXboxController::B(frc::EventLoop* loop) const {
    return frc::GenericHID::Button(m_mapping.GetButton("B"), loop);
}

bool SimXboxController::GetXButton() const {
    return GetRawButton(m_mapping.GetButton("X"));
}
bool SimXboxController::GetXButtonPressed() {
    return GetRawButtonPressed(m_mapping.GetButton("X"));
}
bool SimXboxController::GetXButtonReleased() {
    return GetRawButtonReleased(m_mapping.GetButton("X"));
}

frc::BooleanEvent SimXboxController::X(frc::EventLoop* loop) const {
    return frc::GenericHID::Button(m_mapping.GetButton("X"), loop);
}

bool SimXboxController::GetYButton() const {
    return GetRawButton(m_mapping.GetButton("Y"));
}
bool SimXboxController::GetYButtonPressed() {
    return GetRawButtonPressed(m_mapping.GetButton("Y"));
}
bool SimXboxController::GetYButtonReleased() {
    return GetRawButtonReleased(m_mapping.GetButton("Y"));
}

frc::BooleanEvent SimXboxController::Y(frc::EventLoop* loop) const {
    return frc::GenericHID::Button(m_mapping.GetButton("Y"), loop);
}

bool SimXboxController::GetLeftBumperButton() const {
    return GetRawButton(m_mapping.GetButton("LeftBumper"));
}
bool SimXboxController::GetLeftBumperButtonPressed() {
    return GetRawButtonPressed(m_mapping.GetButton("LeftBumper"));
}

bool SimXboxController::GetLeftBumperButtonReleased() {
    return GetRawButtonReleased(m_mapping.GetButton("LeftBumper"));
}

frc::BooleanEvent SimXboxController::LeftBumper(frc::EventLoop* loop) const {
    return frc::GenericHID::Button(m_mapping.GetButton("LeftBumper"), loop);
}

bool SimXboxController::GetRightBumperButton() const {
    return GetRawButton(m_mapping.GetButton("RightBumper"));
}

bool SimXboxController::GetRightBumperButtonPressed() {
    return GetRawButtonPressed(m_mapping.GetButton("RightBumper"));
}

bool SimXboxController::GetRightBumperButtonReleased() {
    return GetRawButtonReleased(m_mapping.GetButton("RightBumper"));
}

frc::BooleanEvent SimXboxController::RightBumper(frc::EventLoop* loop) const {
    return frc::GenericHID::Button(m_mapping.GetButton("RightBumper"), loop);
}

bool SimXboxController::GetBackButton() const {
    return GetRawButton(m_mapping.GetButton("Back"));
}
bool SimXboxController::GetBackButtonPressed() {
    return GetRawButtonPressed(m_mapping.GetButton("Back"));
}
bool SimXboxController::GetBackButtonReleased() {
    return GetRawButtonReleased(m_mapping.GetButton("Back"));
}

frc::BooleanEvent SimXboxController::Back(frc::EventLoop* loop) const {
    return frc::GenericHID::Button(m_mapping.GetButton("Back"), loop);
}

bool SimXboxController::GetStartButton() const {
    return GetRawButton(m_mapping.GetButton("Start"));
}
bool SimXboxController::GetStartButtonPressed() {
    return GetRawButtonPressed(m_mapping.GetButton("Start"));
}
bool SimXboxController::GetStartButtonReleased() {
    return GetRawButtonReleased(m_mapping.GetButton("Start"));
}

frc::BooleanEvent SimXboxController::Start(frc::EventLoop* loop) const {
    return frc::GenericHID::Button(m_mapping.GetButton("Start"), loop);
}

bool SimXboxController::GetLeftStickButton() const {
    return GetRawButton(m_mapping.GetButton("LeftStick"));
}
bool SimXboxController::GetLeftStickButtonPressed() {
    return GetRawButtonPressed(m_mapping.GetButton("LeftStick"));
}
bool SimXboxController::GetLeftStickButtonReleased() {
    return GetRawButtonReleased(m_mapping.GetButton("LeftStick"));
}

frc::BooleanEvent SimXboxController::LeftStick(frc::EventLoop* loop) const {
    return frc::GenericHID::Button(m_mapping.GetButton("LeftStick"), loop);
}

bool SimXboxController::GetRightStickButton() const {
    return GetRawButton(m_mapping.GetButton("RightStick"));
}
bool SimXboxController::GetRightStickButtonPressed() {
    return GetRawButtonPressed(m_mapping.GetButton("RightStick"));
}

bool SimXboxController::GetRightStickButtonReleased() {
    return GetRawButtonReleased(m_mapping.GetButton("RightStick"));
}

frc::BooleanEvent SimXboxController::RightStick(frc::EventLoop* loop) const {
    return frc::GenericHID::Button(m_mapping.GetButton("RightStick"), loop);
}
