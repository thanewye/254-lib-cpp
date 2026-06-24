#include "lib/util/LoggedTrigger.h"

#include <string>
#include <vector>

#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>

#include "akit/Logger.h"
#include "lib/commands/ChezySequenceCommandGroup.h"
#include "lib/time/RobotTime.h"

LoggedTrigger::LoggedTrigger() : frc2::Trigger() {}

LoggedTrigger::LoggedTrigger(std::function<bool()> condition)
    : frc2::Trigger(std::move(condition)) {}

LoggedTrigger::LoggedTrigger(frc::EventLoop* loop, std::function<bool()> condition)
    : frc2::Trigger(loop, std::move(condition)) {}

LoggedTrigger LoggedTrigger::Wrap(std::function<bool()> condition) {
    return LoggedTrigger(std::move(condition));
}

LoggedTrigger LoggedTrigger::Wrap(frc2::Trigger trigger) {
    return LoggedTrigger([trigger] { return trigger.Get(); });
}

frc2::CommandPtr LoggedTrigger::WrapCommand(frc2::CommandPtr&& command, const std::source_location& loc) {
    std::string cmdName = command.get()->GetName();
    frc2::Command::InterruptionBehavior interruptBehavior = command.get()->GetInterruptionBehavior();
    std::string key = std::string("LoggedTriggers/") + loc.file_name() + "_" +
                      std::to_string(loc.line()) + "/" + cmdName;

    frc2::CommandPtr logCmd = frc2::InstantCommand([key] {
        akit::Logger::RecordOutput(key, robot_time::GetTimestampSeconds());
    }).IgnoringDisable(true);

    std::vector<frc2::CommandPtr> cmds;
    cmds.push_back(std::move(logCmd));
    cmds.push_back(std::move(command));

    return frc2::CommandPtr(std::make_unique<ChezySequenceCommandGroup>(std::move(cmds)))
            .WithName(cmdName)
            .WithInterruptBehavior(interruptBehavior);
}

LoggedTrigger LoggedTrigger::OnTrue(frc2::CommandPtr&& command, std::source_location loc) {
    return Wrap(frc2::Trigger::OnTrue(WrapCommand(std::move(command), loc)));
}

LoggedTrigger LoggedTrigger::OnFalse(frc2::CommandPtr&& command, std::source_location loc) {
    return Wrap(frc2::Trigger::OnFalse(WrapCommand(std::move(command), loc)));
}

LoggedTrigger LoggedTrigger::WhileTrue(frc2::CommandPtr&& command, std::source_location loc) {
    return Wrap(frc2::Trigger::WhileTrue(WrapCommand(std::move(command), loc)));
}

LoggedTrigger LoggedTrigger::WhileFalse(frc2::CommandPtr&& command, std::source_location loc) {
    return Wrap(frc2::Trigger::WhileFalse(WrapCommand(std::move(command), loc)));
}

LoggedTrigger LoggedTrigger::ToggleOnTrue(frc2::CommandPtr&& command, std::source_location loc) {
    return Wrap(frc2::Trigger::ToggleOnTrue(WrapCommand(std::move(command), loc)));
}

LoggedTrigger LoggedTrigger::ToggleOnFalse(frc2::CommandPtr&& command, std::source_location loc) {
    return Wrap(frc2::Trigger::ToggleOnFalse(WrapCommand(std::move(command), loc)));
}

LoggedTrigger LoggedTrigger::operator&&(frc2::Trigger rhs) const {
    return LoggedTrigger([self = *this, rhs] { return self.Get() && rhs.Get(); });
}

LoggedTrigger LoggedTrigger::operator||(frc2::Trigger rhs) const {
    return LoggedTrigger([self = *this, rhs] { return self.Get() || rhs.Get(); });
}

LoggedTrigger LoggedTrigger::operator!() const {
    return LoggedTrigger([self = *this] { return !self.Get(); });
}

LoggedTrigger LoggedTrigger::Debounce(units::second_t debounceTime, frc::Debouncer::DebounceType type) {
    frc2::Trigger wrapped = frc2::Trigger::Debounce(debounceTime, type);
    return LoggedTrigger([wrapped] { return wrapped.Get(); });
}
