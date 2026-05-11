#pragma once

#include <functional>
#include <source_location>

#include <frc/event/EventLoop.h>
#include <frc/filter/Debouncer.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>
#include <units/time.h>

class LoggedTrigger : public frc2::Trigger {
public:
    LoggedTrigger();
    explicit LoggedTrigger(std::function<bool()> condition);
    LoggedTrigger(frc::EventLoop* loop, std::function<bool()> condition);

    static LoggedTrigger Wrap(std::function<bool()> condition);
    static LoggedTrigger Wrap(frc2::Trigger trigger);

    frc2::Trigger OnTrue(frc2::CommandPtr&& command,
                         std::source_location loc = std::source_location::current());
    frc2::Trigger OnFalse(frc2::CommandPtr&& command,
                          std::source_location loc = std::source_location::current());
    frc2::Trigger WhileTrue(frc2::CommandPtr&& command,
                            std::source_location loc = std::source_location::current());
    frc2::Trigger WhileFalse(frc2::CommandPtr&& command,
                             std::source_location loc = std::source_location::current());
    frc2::Trigger ToggleOnTrue(frc2::CommandPtr&& command,
                               std::source_location loc = std::source_location::current());
    frc2::Trigger ToggleOnFalse(frc2::CommandPtr&& command,
                                std::source_location loc = std::source_location::current());

    LoggedTrigger operator&&(frc2::Trigger rhs) const;
    LoggedTrigger operator||(frc2::Trigger rhs) const;
    LoggedTrigger operator!() const;

    LoggedTrigger Debounce(units::second_t debounceTime,
                           frc::Debouncer::DebounceType type = frc::Debouncer::DebounceType::kRising);

private:
    frc2::CommandPtr WrapCommand(frc2::CommandPtr&& command, const std::source_location& loc);
};