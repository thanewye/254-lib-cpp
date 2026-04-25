#include "lib/util/StatusSignalManager.h"
#include "akit/Logger.h"
#include "ctre/phoenix6/StatusSignal.hpp"
#include "frc/Errors.h"
#include "frc/Timer.h"

void StatusSignalManager::Register(ctre::phoenix6::BaseStatusSignal* signal) {
    signals.push_back(signal);
}

void StatusSignalManager::RefreshAll() {
    double timestamp = frc::Timer::GetFPGATimestamp().value();
    if (signals.empty()) return;
    auto status = ctre::phoenix6::BaseStatusSignal::RefreshAll(signals);
    if (status != lastStatus) {
        lastStatus = status;
        if (status != ctre::phoenix::StatusCode::OK) {
            FRC_ReportError(frc::err::Error,
                            "StatusSignalManager: Failed to refresh signals: {}",
                            status.GetName());
        } else {
            FRC_ReportWarning("StatusSignalManager: Signal refresh recovered: {}", status.GetName());
        }
    }
    akit::Logger::RecordOutputs("StatusSignalManager/LastStatus", status.GetName());
    akit::Logger::RecordOutputs("StatusSignalManager/latencyPeriodSec", frc::Timer::GetFPGATimestamp().value() - timestamp);
}

StatusSignalManager& StatusSignalManager::GetInstance() {
    static StatusSignalManager instance;
    return instance;
}