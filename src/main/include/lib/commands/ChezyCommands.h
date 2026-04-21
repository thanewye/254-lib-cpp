#pragma once

#include <frc2/command/CommandPtr.h>
#include <vector>

namespace ChezyCommands {
  frc2::CommandPtr Sequence(std::vector<frc2::CommandPtr> commands);
}
