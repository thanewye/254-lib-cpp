#pragma once

#include <functional>
#include <string>

#include <frc2/command/button/Trigger.h>

class LoggedTrigger : public frc2::Trigger {
 public:
  LoggedTrigger();
  explicit LoggedTrigger(std::function<bool()> condition);

  Trigger OnTrue(frc2::CommandPtr command) &&;
  Trigger OnFalse(frc2::CommandPtr command) &&;
  Trigger WhileTrue(frc2::CommandPtr command) &&;
  Trigger WhileFalse(frc2::CommandPtr command) &&;
  LoggedTrigger operator&&(Trigger other) const;
  LoggedTrigger operator||(Trigger other) const;
  LoggedTrigger operator!() const;
};
