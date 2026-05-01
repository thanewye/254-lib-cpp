#pragma once

#include <functional>

#include <frc2/command/button/Trigger.h>

class CheesyTrigger : public frc2::Trigger {
 public:
  explicit CheesyTrigger(std::function<bool()> condition);

  Trigger WhileTrueAlwaysRunning(frc2::CommandPtr command);
};
