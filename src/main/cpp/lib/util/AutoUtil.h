#pragma once

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include <frc/filesystem.h>

namespace auto_util {
    namespace fs = std::filesystem;
    std::vector<std::string> GetAutoListNoExtension() {
        fs::path deployDir = frc::filesystem::GetDeployDirectory();
        fs::path choreoDir = deployDir / "choreo";

        std::vector<std::string> autoNames;

        for (const auto& file : fs::directory_iterator(choreoDir)) {
            if (file.path().extension() != ".traj") continue;
            autoNames.push_back(file.path().stem().string());
        }
        std::ranges::sort(autoNames);
        autoNames.erase(std::ranges::unique(autoNames).begin(), autoNames.end());
        return autoNames;
    }
} // namespace auto_util
