#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>

namespace MathHelpers {
  frc::Pose2d FlipForAlliance(const frc::Pose2d& pose);
  frc::Translation2d FlipForAlliance(const frc::Translation2d& translation);
  units::radian_t WrapAngle(units::radian_t angle);
  double Deadband(double value, double deadband);
}
