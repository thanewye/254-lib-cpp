#include "lib/util/CheesyTrigger.h"

#include <memory>
#include <utility>

#include <frc2/command/CommandScheduler.h>

#include "lib/util/Errors.h"

CheesyTrigger::CheesyTrigger(frc::EventLoop* loop, const std::function<bool()>& condition)
    : Trigger(loop, condition)
    , loop(errors::RequireNonNullParam(loop, "loop", "Trigger")) {}

CheesyTrigger::CheesyTrigger(const std::function<bool()>& condition)
    : Trigger(frc2::CommandScheduler::GetInstance().GetActiveButtonLoop(), condition)
    , condition(condition)
    , loop(frc2::CommandScheduler::GetInstance().GetActiveButtonLoop()) {}

CheesyTrigger& CheesyTrigger::WhileTrueAlwaysRunning(frc2::CommandPtr command) {
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

void CheesyTrigger::AddBinding(const BindingBody& body) {
    bool previous = condition();
    loop->Bind([this, body, previous]() mutable {
        const bool current = condition();
        body(previous, current);
        previous = current;
    });
}
