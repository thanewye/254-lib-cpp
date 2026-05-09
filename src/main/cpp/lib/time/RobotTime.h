#pragma once

#include <frc/Timer.h>
#include <units/time.h>

namespace RobotTime {
    inline double GetTimestampSeconds() {
        return frc::Timer::GetTimestamp().value();
    }

    inline units::second_t GetTimestamp() {
        return frc::Timer::GetTimestamp();
    }
}
