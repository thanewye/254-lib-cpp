#pragma once

#include <functional>
#include <utility>

#include <frc2/command/button/Trigger.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/event/EventLoop.h>
#include <frc2/command/CommandPtr.h>

#include "Errors.h"

class CheesyTrigger : public frc2::Trigger {
    using BindingBody = std::function<void(bool previous, bool current)>;
protected:
    std::function<bool()> condition;
    frc::EventLoop* loop;
    bool shouldBeRunning = false;

    void AddBinding(const CheesyTrigger::BindingBody& body) {
        bool previous = condition();
        loop->Bind([this, body, previous] () mutable{
            const bool current = condition();
            body(previous, current);
            previous = current;
        });
    }

public:
    explicit CheesyTrigger(frc::EventLoop* loop, const std::function<bool()>& condition)
        : Trigger(loop, condition)
        , loop(Errors::RequireNonNullParam(loop, "loop", "Trigger")){}

    explicit CheesyTrigger(const std::function<bool()>& condition)
        : Trigger(frc2::CommandScheduler::GetInstance().GetActiveButtonLoop(), condition)
        , condition(condition)
        , loop(frc2::CommandScheduler::GetInstance().GetActiveButtonLoop()) {}

    CheesyTrigger& WhileTrueAlwaysRunning(frc2::CommandPtr command) {
        auto owned = std::make_shared<frc2::CommandPtr>(std::move(command));
        frc2::Command* raw = owned->get();
        AddBinding(
            [this, raw, owned](bool previous, bool current) {
                (void) owned; // owned is captured to keep raw alive
                if (!previous && current) {
                    frc2::CommandScheduler::GetInstance().Schedule(raw);
                    shouldBeRunning = true;
                } else if (previous && !current) {
                    raw->Cancel();
                    shouldBeRunning = false;
                }
                if (current && shouldBeRunning && !raw->IsScheduled()) {
                    frc2::CommandScheduler::GetInstance().Schedule(raw);
                }
            });
        return *this;
    }
};
