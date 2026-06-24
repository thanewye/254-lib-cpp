#pragma once

#include <vector>

#include <frc2/command/CommandPtr.h>

namespace chezy_commands {
  frc2::CommandPtr Sequence(std::vector<frc2::CommandPtr> commands);
} // namespace chezy_commands
