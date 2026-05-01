#pragma once

#include <algorithm>
#include <cmath>
#include <functional>

#include <frc/geometry/Pose2d.h>

#include "lib/util/FieldConstants.h"

namespace Util {
    inline constexpr double kEpsilon = 1e-12;

    inline double Deadband(double value, double deadband) {
        deadband = std::abs(deadband);
        if (deadband == 1.0) {
            return 0.0;
        }

        double scaledValue =
                (value + (value < 0.0 ? deadband : -deadband)) / (1.0 - deadband);
        return std::abs(value) > deadband ? scaledValue : 0.0;
    }

    inline double Limit(double value, double limit) {
        return std::clamp(value, -limit, limit);
    }

    inline double Interpolate(double a, double b, double t) {
        t = std::clamp(t, 0.0, 1.0);
        return a + (b - a) * t;
    }

    inline bool EpsilonEquals(double a, double b, double epsilon = kEpsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    inline frc::Pose2d FlipPoseForAlliance(const frc::Pose2d &pose) {
        return frc::Pose2d{
            FieldConstants::kFieldLength - pose.X(),
            pose.Y(),
            pose.Rotation()
        };
    }

    template<typename T>
    std::function<T()> Memoize(std::function<T()> fn);
}
