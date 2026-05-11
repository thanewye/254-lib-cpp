#pragma once

#include <functional>
#include <frc/Timer.h>

#include <frc2/command/button/Trigger.h>
#include <units/current.h>

class CurrentSpikeDetector {
public:
    CurrentSpikeDetector(double thresholdAmps, double durationSeconds)
        : currentThresholdAmps(static_cast<units::ampere_t>(thresholdAmps))
          , timeThresholdSeconds(static_cast<units::second_t>(durationSeconds))
          , currentOverThresholdTimer(frc::Timer()) {}

    bool Update(units::ampere_t currentAmps) {
        if (currentAmps > currentThresholdAmps) {
            currentOverThresholdTimer.Start();
            bool newVal = currentOverThresholdTimer.HasElapsed(timeThresholdSeconds);
            lastValue = newVal;
            return newVal;
        } else {
            currentOverThresholdTimer.Stop();
            currentOverThresholdTimer.Reset();
            lastValue = false;
            return false;
        }
    }
    bool operator()() const {
        return lastValue;
    }

    frc2::Trigger AsTrigger() {
        if (!cachedTrigger.has_value()) {
            cachedTrigger = frc2::Trigger(std::ref(*this));
        }
        return *cachedTrigger;
    }

private:
    units::ampere_t currentThresholdAmps;
    units::second_t timeThresholdSeconds;
    frc::Timer currentOverThresholdTimer;
    std::optional<frc2::Trigger> cachedTrigger;
    bool lastValue = false;
};
