#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <frc/RobotBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include "akit/Logger.h"

namespace log_util {
    inline bool ShouldLog() {
        return !frc::RobotBase::IsReal();
    }

    inline void Log(const std::function<void()>& fn) {
        if (ShouldLog()) {
            fn();
        }
    }

    // Primitives
    inline void RecordOutput(const std::string& key, double value) {
        if (ShouldLog()) {
            akit::Logger::RecordOutput(key, value);
        }
    }

    inline void RecordOutput(const std::string& key, const std::vector<double>& values) {
        if (ShouldLog()) {
            akit::Logger::RecordOutput(key, values);
        }
    }

    inline void RecordOutput(const std::string& key, int64_t value) {
        if (ShouldLog()) {
            akit::Logger::RecordOutput(key, value);
        }
    }

    inline void RecordOutput(const std::string& key, bool value) {
        if (ShouldLog()) {
            akit::Logger::RecordOutput(key, value);
        }
    }

    inline void RecordOutput(const std::string& key, const std::string& value) {
        if (ShouldLog()) {
            akit::Logger::RecordOutput(key, value);
        }
    }

    // Pose2d
    inline void RecordOutput(const std::string& key, const frc::Pose2d& pose) {
        if (ShouldLog()) {
            akit::Logger::RecordOutput(key, std::vector<double>{pose.X().value(), pose.Y().value(), pose.Rotation().Radians().value()});
        }
    }

    inline void RecordOutput(const std::string& key, const std::vector<frc::Pose2d>& poses) {
        if (!ShouldLog()) {
            return;
        }
        std::vector<double> packed;
        packed.reserve(poses.size() * 3);
        for (const auto& pose : poses) {
            packed.push_back(pose.X().value());
            packed.push_back(pose.Y().value());
            packed.push_back(pose.Rotation().Radians().value());
        }
        akit::Logger::RecordOutput(key, std::move(packed));
    }

    // Pose3d
    inline void RecordOutput(const std::string& key, const frc::Pose3d& pose) {
        if (ShouldLog()) {
            akit::Logger::RecordOutput(key, std::vector<double>{pose.X().value(), pose.Y().value(), pose.Z().value(), pose.Rotation().X().value(),
                                                                pose.Rotation().Y().value(), pose.Rotation().Z().value()});
        }
    }

    inline void RecordOutput(const std::string& key, const std::vector<frc::Pose3d>& poses) {}

    // Translation2d
    inline void RecordOutput(const std::string& key, const frc::Translation2d& translation) {}
    inline void RecordOutput(const std::string& key, const std::vector<frc::Translation2d>& translations) {}

    // Translation3d
    inline void RecordOutput(const std::string& key, const frc::Translation3d& translation) {}
    inline void RecordOutput(const std::string& key, const std::vector<frc::Translation3d>& translations) {}

    // Rotation2d
    inline void RecordOutput(const std::string& key, const frc::Rotation2d& rotation) {}
    inline void RecordOutput(const std::string& key, const std::vector<frc::Rotation2d>& rotations) {}

    // Rotation3d
    inline void RecordOutput(const std::string& key, const frc::Rotation3d& rotation) {}
    inline void RecordOutput(const std::string& key, const std::vector<frc::Rotation3d>& rotations) {}

    // Transform2d
    inline void RecordOutput(const std::string& key, const frc::Transform2d& transform) {}
    inline void RecordOutput(const std::string& key, const std::vector<frc::Transform2d>& transforms) {}

    // Transform3d
    inline void RecordOutput(const std::string& key, const frc::Transform3d& transform) {}
    inline void RecordOutput(const std::string& key, const std::vector<frc::Transform3d>& transforms) {}

    // ChassisSpeeds
    inline void RecordOutput(const std::string& key, const frc::ChassisSpeeds& speeds) {}
    inline void RecordOutput(const std::string& key, const std::vector<frc::ChassisSpeeds>& speeds) {}

    // SwerveModuleState
    inline void RecordOutput(const std::string& key, const frc::SwerveModuleState& state) {}
    inline void RecordOutput(const std::string& key, const std::vector<frc::SwerveModuleState>& states) {}

    // SwerveModulePosition
    inline void RecordOutput(const std::string& key, const frc::SwerveModulePosition& position) {}
    inline void RecordOutput(const std::string& key, const std::vector<frc::SwerveModulePosition>& positions) {}
} // namespace log_util
