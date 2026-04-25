#pragma once

#include <frc/geometry/Pose2d.h>

class ExponentialMovingAveragePose {
 public:
  explicit ExponentialMovingAveragePose(double alpha);

  frc::Pose2d Calculate(const frc::Pose2d& pose);
  void Reset();

 private:
  double m_alpha;
  frc::Pose2d m_average;
  bool m_initialized = false;
};
