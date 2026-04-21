#pragma once

#include <frc2/command/button/Trigger.h>
#include <functional>
#include <string>

class LoggedTrigger : public frc2::Trigger {
 public:
  LoggedTrigger();
  explicit LoggedTrigger(std::function<bool()> condition);

  frc2::Trigger OnTrue(frc2::CommandPtr command) &&;
  frc2::Trigger OnFalse(frc2::CommandPtr command) &&;
  frc2::Trigger WhileTrue(frc2::CommandPtr command) &&;
  frc2::Trigger WhileFalse(frc2::CommandPtr command) &&;
  LoggedTrigger operator&&(frc2::Trigger other) const;
  LoggedTrigger operator||(frc2::Trigger other) const;
  LoggedTrigger operator!() const;
};
