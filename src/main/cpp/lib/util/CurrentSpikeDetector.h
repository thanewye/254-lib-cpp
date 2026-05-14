#pragma once

#include <optional>

#include <frc/Timer.h>
#include <frc2/command/button/Trigger.h>
#include <units/current.h>

class CurrentSpikeDetector {
public:
    CurrentSpikeDetector(double thresholdAmps, double durationSeconds);

    bool Update(units::ampere_t currentAmps);
    bool operator()() const;
    frc2::Trigger AsTrigger();

private:
    units::ampere_t currentThresholdAmps;
    units::second_t timeThresholdSeconds;
    frc::Timer currentOverThresholdTimer;
    std::optional<frc2::Trigger> cachedTrigger;
    bool lastValue = false;
};
