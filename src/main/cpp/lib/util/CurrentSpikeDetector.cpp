#include "lib/util/CurrentSpikeDetector.h"

#include <functional>

CurrentSpikeDetector::CurrentSpikeDetector(double thresholdAmps, double durationSeconds)
    : currentThresholdAmps(static_cast<units::ampere_t>(thresholdAmps))
    , timeThresholdSeconds(static_cast<units::second_t>(durationSeconds))
    , currentOverThresholdTimer(frc::Timer()) {}

bool CurrentSpikeDetector::Update(units::ampere_t currentAmps) {
    if (currentAmps > currentThresholdAmps) {
        currentOverThresholdTimer.Start();
        bool newVal = currentOverThresholdTimer.HasElapsed(timeThresholdSeconds);
        lastValue = newVal;
        return newVal;
    }
    currentOverThresholdTimer.Stop();
    currentOverThresholdTimer.Reset();
    lastValue = false;
    return false;
}

bool CurrentSpikeDetector::operator()() const {
    return lastValue;
}

frc2::Trigger CurrentSpikeDetector::AsTrigger() {
    if (!cachedTrigger.has_value()) {
        cachedTrigger = frc2::Trigger(std::ref(*this));
    }
    return *cachedTrigger;
}
