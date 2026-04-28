#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <memory>

class ChezyRepeatCommand : public frc2::Command {
 public:
  explicit ChezyRepeatCommand(frc2::CommandPtr command);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  bool RunsWhenDisabled() const override;

 private:
  std::unique_ptr<Command> m_command;
};
