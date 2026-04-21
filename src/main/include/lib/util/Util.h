#pragma once

#include <frc/geometry/Pose2d.h>
#include <functional>

namespace Util {
  inline constexpr double kEpsilon = 1e-12;

  double Deadband(double value, double deadband);
  double Limit(double value, double limit);
  double Interpolate(double a, double b, double t);
  bool EpsilonEquals(double a, double b, double epsilon = kEpsilon);
  frc::Pose2d FlipPoseForAlliance(const frc::Pose2d& pose);

  template <typename T>
  std::function<T()> Memoize(std::function<T()> fn);
}
