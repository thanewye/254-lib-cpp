#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <memory>

class ChezyProxyCommand : public frc2::Command {
 public:
  explicit ChezyProxyCommand(frc2::CommandPtr command);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  bool RunsWhenDisabled() const override;

 private:
  std::unique_ptr<frc2::Command> m_command;
};
