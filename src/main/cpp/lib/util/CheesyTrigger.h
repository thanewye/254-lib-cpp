#pragma once

#include <functional>

#include <frc2/command/button/Trigger.h>
#include <frc/event/EventLoop.h>
#include <frc2/command/CommandPtr.h>

class CheesyTrigger : public frc2::Trigger {
public:
    explicit CheesyTrigger(frc::EventLoop* loop, const std::function<bool()>& condition);
    explicit CheesyTrigger(const std::function<bool()>& condition);

    CheesyTrigger& WhileTrueAlwaysRunning(frc2::CommandPtr command);

protected:
    using BindingBody = std::function<void(bool previous, bool current)>;

    std::function<bool()> condition;
    frc::EventLoop* loop;
    bool shouldBeRunning = false;

    void AddBinding(const BindingBody& body);
};
