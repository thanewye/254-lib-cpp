#pragma once

#include <frc2/command/button/CommandXboxController.h>

#include "lib/util/ControllerMapping.h"

class CommandSimXboxController : public frc2::CommandXboxController {
 public:
  explicit CommandSimXboxController(int port);

  double GetLeftX() const;
  double GetLeftY() const;
  double GetRightX() const;
  double GetRightY() const;
  double GetLeftTriggerAxis() const;
  double GetRightTriggerAxis() const;
};
