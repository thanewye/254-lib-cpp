#include "lib/commands/ChezySequenceCommandGroup.h"

#include <stdexcept>

void ChezySequenceCommandGroup::AddCommands(std::vector<frc2::CommandPtr> commands) {
    if (currentCommandIndex != -1) {
        throw std::runtime_error("Commands cannot be added to a composition while it's running");
    }
    for (auto& cmd : commands) {
        AddRequirements(cmd.get()->GetRequirements());
        runWhenDisabled &= cmd.get()->RunsWhenDisabled();
        if (cmd.get()->GetInterruptionBehavior() == InterruptionBehavior::kCancelSelf) {
            interruptBehavior = InterruptionBehavior::kCancelSelf;
        }
        commands_.push_back(std::move(cmd).Unwrap());
    }
}

void ChezySequenceCommandGroup::Initialize() {
    currentCommandIndex = 0;
    if (!commands_.empty()) {
        commands_[0]->Initialize();
    }
}

void ChezySequenceCommandGroup::Execute() {
    if (commands_.empty()) {
        return;
    }
    auto& current = commands_[currentCommandIndex];
    current->Execute();
    if (current->IsFinished()) {
        current->End(false);
        ++currentCommandIndex;
        if (currentCommandIndex < static_cast<int>(commands_.size())) {
            commands_[currentCommandIndex]->Initialize();
            Execute();
        }
    }
}

void ChezySequenceCommandGroup::End(bool interrupted) {
    if (interrupted && !commands_.empty() && currentCommandIndex > -1 && currentCommandIndex < static_cast<int>(commands_.size())) {
        commands_[currentCommandIndex]->End(true);
    }
    currentCommandIndex = -1;
}
