#pragma once

#include <frc/Timer.h>
#include <units/time.h>

namespace robot_time {
    inline double GetTimestampSeconds() {
        return frc::Timer::GetTimestamp().value();
    }

    inline units::second_t GetTimestamp() {
        return frc::Timer::GetTimestamp();
    }
} // namespace robot_time
