#pragma once

#include <functional>

#include <frc2/command/button/Trigger.h>

class CurrentSpikeDetector {
 public:
  CurrentSpikeDetector(double thresholdAmps, double durationSeconds);

  bool Update(double currentAmps);
  bool GetAsBoolean() const;
  frc2::Trigger AsTrigger();

 private:
  double m_threshold;
  double m_duration;
  bool m_triggered = false;
};
