#pragma once

#include <frc/XboxController.h>

#include "lib/util/ControllerMapping.h"

class SimXboxController : public frc::XboxController {
 public:
  SimXboxController(int port, const ControllerMapping& mapping);

  double GetLeftX() const;
  double GetLeftY() const;
  double GetRightX() const;
  double GetRightY() const;
  double GetLeftTriggerAxis() const;
  double GetRightTriggerAxis() const;

 private:
  const ControllerMapping& m_mapping;
};
