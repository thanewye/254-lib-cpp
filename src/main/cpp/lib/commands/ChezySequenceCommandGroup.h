#pragma once

#include <concepts>
#include <memory>
#include <vector>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>

class ChezySequenceCommandGroup
        : public frc2::CommandHelper<frc2::Command, ChezySequenceCommandGroup> {
public:
    explicit ChezySequenceCommandGroup(std::vector<frc2::CommandPtr> commands) {
        AddCommands(std::move(commands));
    }

    template<typename... Cmds>
        requires (std::convertible_to<Cmds, frc2::CommandPtr> && ...)
    explicit ChezySequenceCommandGroup(Cmds&&... cmds) {
        std::vector<frc2::CommandPtr> v;
        (v.emplace_back(std::forward<Cmds>(cmds)), ...);
        AddCommands(std::move(v));
    }

    template<typename... Cmds>
        requires (std::convertible_to<Cmds, frc2::CommandPtr> && ...)
    void AddCommands(Cmds&&... cmds) {
        std::vector<frc2::CommandPtr> v;
        (v.emplace_back(std::forward<Cmds>(cmds)), ...);
        AddCommands(std::move(v));
    }

    void AddCommands(std::vector<frc2::CommandPtr> commands);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;

    bool IsFinished() override {
        return currentCommandIndex == static_cast<int>(commands_.size());
    }

    bool RunsWhenDisabled() const override {
        return runWhenDisabled;
    }

    InterruptionBehavior GetInterruptionBehavior() const override {
        return interruptBehavior;
    }

private:
    std::vector<std::unique_ptr<frc2::Command>> commands_;
    int currentCommandIndex = -1;
    bool runWhenDisabled = true;
    InterruptionBehavior interruptBehavior = InterruptionBehavior::kCancelIncoming;
};
