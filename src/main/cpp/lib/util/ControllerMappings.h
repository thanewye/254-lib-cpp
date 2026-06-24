#pragma once

#include "lib/util/ControllerMapping.h"

namespace controller_mappings {
    const auto XBOX_MAPPING = ControllerMapping(
        {
            {"A", 1}, {"B", 2}, {"X", 4}, {"Y", 5},
            {"LeftBumper", 7}, {"RightBumper", 8},
            {"Back", 11}, {"Start", 12},
            {"LeftStick", 14}, {"RightStick", 15}
        },
        {
            {"LeftX", 0}, {"LeftY", 1}, {"RightX", 2}, {"RightY", 3},
            {"RightTrigger", 4}, {"LeftTrigger", 5}
        }
    );

    const auto DUALSENSE_MAPPING = ControllerMapping(
        {
            {"A", 1}, {"B", 2}, {"X", 3}, {"Y", 4},
            {"LeftBumper", 5}, {"RightBumper", 6},
            {"Back", 7}, {"Start", 8},
            {"LeftStick", 10}, {"RightStick", 11}
        },
        {
            {"LeftX", 0}, {"LeftY", 1}, {"RightX", 4}, {"RightY", 5},
            {"LeftTrigger", 2}, {"RightTrigger", 3}
        }
    );
} // namespace controller_mappings
