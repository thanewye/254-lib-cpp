#pragma once

#include <vector>

#include <frc2/command/CommandPtr.h>

namespace ChezyCommands {
  frc2::CommandPtr Sequence(std::vector<frc2::CommandPtr> commands);
}
