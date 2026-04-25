#pragma once

#include <frc2/command/button/Trigger.h>
#include <functional>

class CheesyTrigger : public frc2::Trigger {
 public:
  explicit CheesyTrigger(std::function<bool()> condition);

  frc2::Trigger WhileTrueAlwaysRunning(frc2::CommandPtr command);
};
