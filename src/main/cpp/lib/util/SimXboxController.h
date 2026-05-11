#pragma once

#include <frc/XboxController.h>

#include "lib/util/ControllerMapping.h"

class SimXboxController : public frc::XboxController {
public:
    SimXboxController(int port, const ControllerMapping& mapping);

    double GetLeftX() const;
    double GetRightX() const;
    double GetLeftY() const;
    double GetRightY() const;

    double GetLeftTriggerAxis() const;
    frc::BooleanEvent LeftTrigger(double threshold, frc::EventLoop* loop) const;
    frc::BooleanEvent LeftTrigger(frc::EventLoop* loop) const;

    double GetRightTriggerAxis() const;
    frc::BooleanEvent RightTrigger(double threshold, frc::EventLoop* loop) const;
    frc::BooleanEvent RightTrigger(frc::EventLoop* loop) const;

    bool GetAButton() const;
    bool GetAButtonPressed();
    bool GetAButtonReleased();
    frc::BooleanEvent A(frc::EventLoop* loop) const;

    bool GetBButton() const;
    bool GetBButtonPressed();
    bool GetBButtonReleased();
    frc::BooleanEvent B(frc::EventLoop* loop) const;

    bool GetXButton() const;
    bool GetXButtonPressed();
    bool GetXButtonReleased();
    frc::BooleanEvent X(frc::EventLoop* loop) const;

    bool GetYButton() const;
    bool GetYButtonPressed();
    bool GetYButtonReleased();
    frc::BooleanEvent Y(frc::EventLoop* loop) const;

    bool GetLeftBumperButton() const;
    bool GetLeftBumperButtonPressed();
    bool GetLeftBumperButtonReleased();
    frc::BooleanEvent LeftBumper(frc::EventLoop* loop) const;

    bool GetRightBumperButton() const;
    bool GetRightBumperButtonPressed();
    bool GetRightBumperButtonReleased();
    frc::BooleanEvent RightBumper(frc::EventLoop* loop) const;

    bool GetBackButton() const;
    bool GetBackButtonPressed();
    bool GetBackButtonReleased();
    frc::BooleanEvent Back(frc::EventLoop* loop) const;

    bool GetStartButton() const;
    bool GetStartButtonPressed();
    bool GetStartButtonReleased();
    frc::BooleanEvent Start(frc::EventLoop* loop) const;

    bool GetLeftStickButton() const;
    bool GetLeftStickButtonPressed();
    bool GetLeftStickButtonReleased();
    frc::BooleanEvent LeftStick(frc::EventLoop* loop) const;

    bool GetRightStickButton() const;
    bool GetRightStickButtonPressed();
    bool GetRightStickButtonReleased();
    frc::BooleanEvent RightStick(frc::EventLoop* loop) const;

private:
    const ControllerMapping& m_mapping;
};
