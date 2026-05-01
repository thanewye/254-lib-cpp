#pragma once

#include <memory>
#include <vector>

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>

class ChezySequenceCommandGroup : public frc2::Command {
 public:
  explicit ChezySequenceCommandGroup(std::vector<frc2::CommandPtr> commands);

  void AddCommands(std::vector<frc2::CommandPtr> commands);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  bool RunsWhenDisabled() const override;

 private:
  std::vector<std::unique_ptr<Command>> m_commands;
  size_t m_currentIndex = 0;
};
