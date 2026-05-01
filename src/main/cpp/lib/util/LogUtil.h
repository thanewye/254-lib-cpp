#pragma once

#include <functional>
#include <string>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>

namespace LogUtil {
  inline constexpr bool kShouldLog = true;

  void Log(std::function<void()> fn);
  void RecordOutput(const std::string& key, double value);
  void RecordOutput(const std::string& key, bool value);
  void RecordOutput(const std::string& key, const frc::Pose2d& pose);
  void RecordOutput(const std::string& key, const std::vector<frc::Pose2d>& poses);
  void RecordOutput(const std::string& key, const frc::Pose3d& pose);
}
