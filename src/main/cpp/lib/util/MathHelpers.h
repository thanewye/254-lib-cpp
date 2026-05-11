#pragma once

#include <algorithm>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/angle.h>

namespace MathHelpers {
    inline constexpr frc::Pose2d kPose2dZero{};
    inline constexpr frc::Rotation2d kRotation2dZero{};
    inline constexpr frc::Rotation2d kRotation2dPi = frc::Rotation2d{180_deg};
    inline constexpr frc::Translation2d kTranslation2dZero{};
    inline constexpr frc::Transform2d kTransform2dZero{};

    inline frc::Pose2d Pose2dFromRotation(const frc::Rotation2d &rotation) {
        return frc::Pose2d{kTranslation2dZero, rotation};
    }

    inline frc::Pose2d Pose2dFromTranslation(const frc::Translation2d &translation) {
        return frc::Pose2d{translation, kRotation2dZero};
    }

    inline frc::Transform2d Transform2dFromRotation(const frc::Rotation2d &rotation) {
        return frc::Transform2d{kTranslation2dZero, rotation};
    }

    inline frc::Transform2d Transform2dFromTranslation(const frc::Translation2d &translation) {
        return frc::Transform2d{translation, kRotation2dZero};
    }

    template<typename T>
    inline T Signum(T value) {
        if (value > T{0}) return T{1};
        if (value < T{0}) return T{-1};
        return T{0};
    }

    inline bool TranslationWithin(
        const frc::Translation2d &query,
        const frc::Translation2d &cornerA,
        const frc::Translation2d &cornerB) {
        const double minX = std::min(cornerA.X().value(), cornerB.X().value());
        const double maxX = std::max(cornerA.X().value(), cornerB.X().value());
        const double minY = std::min(cornerA.Y().value(), cornerB.Y().value());
        const double maxY = std::max(cornerA.Y().value(), cornerB.Y().value());
        return query.X().value() >= minX && query.X().value() <= maxX &&
               query.Y().value() >= minY && query.Y().value() <= maxY;
    }

    inline bool PoseWithin(
        const frc::Pose2d &query,
        const frc::Pose2d &cornerA,
        const frc::Pose2d &cornerB) {
        return TranslationWithin(query.Translation(), cornerA.Translation(), cornerB.Translation());
    }

    inline double ReverseInterpolate(
        const frc::Translation2d &query,
        const frc::Translation2d &start,
        const frc::Translation2d &end) {
        const frc::Translation2d segment = end - start;
        const frc::Translation2d queryToStart = query - start;

        const double segmentLengthSqr = segment.X().value() * segment.X().value() +
                                        segment.Y().value() * segment.Y().value();
        if (segmentLengthSqr == 0.0) {
            return 0.0;
        }

        return (queryToStart.X().value() * segment.X().value() +
                queryToStart.Y().value() * segment.Y().value()) /
               segmentLengthSqr;
    }

    inline double DistanceToLineSegment(
        const frc::Translation2d &query,
        const frc::Translation2d &start,
        const frc::Translation2d &end) {
        const double t = ReverseInterpolate(query, start, end);
        if (t < 0.0) {
            return query.Distance(start).value();
        }
        if (t > 1.0) {
            return query.Distance(end).value();
        }

        const frc::Translation2d segment = end - start;
        const frc::Translation2d closestPoint = start + segment * t;
        return query.Distance(closestPoint).value();
    }

    inline double PerpendicularDistanceToLine(
        const frc::Translation2d &query,
        const frc::Translation2d &start,
        const frc::Translation2d &end) {
        const double t = ReverseInterpolate(query, start, end);
        const frc::Translation2d segment = end - start;
        const frc::Translation2d closestPoint = start + segment * t;
        return query.Distance(closestPoint).value();
    }
}
