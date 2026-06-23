#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <span>
#include <utility>

#include <frc/Timer.h>

#include "akit/Logger.h"

#include <frc/RobotController.h>

#include "akit/telemetry/LoggedDriverStation.h"
#include "akit/telemetry/LoggedPowerDistribution.h"
#include "akit/LoggedRobot.h"
#include "akit/telemetry/LoggedSystemStats.h"
#include "akit/telemetry/RadioLogger.h"

namespace akit {
    void Logger::Start() {
        if (running_) return;

        if (!LoggedRobot::IsBaseConstructed()) {
            FRC_ReportError(
                frc::err::Error,
                "The main robot class must inherit from LoggedRobot when using AdvantageKit. For more details, check the AdvantageKit installation documentation: https://docs.advantagekit.org/getting-started/installation\n\n*** EXITING DUE TO INVALID ADVANTAGEKIT INSTALLATION, SEE ABOVE. ***");
            return;
        }

        if (IsReplayMode()) {
            const char* halSimExtensions = std::getenv("HALSIM_EXTENSIONS");
            if (halSimExtensions != nullptr && halSimExtensions[0] != '\0') {
                FRC_ReportError(
                    frc::err::Error,
                    "[AdvantageKit] All HAL simulation extensions must be disabled when running AdvantageKit replay, including the simulation GUI and DriverStation connection. Check the configuration in \"build.gradle\" and ensure that all checkboxes are disabled in the VSCode simulation popup.\n\n*** EXITING DUE TO INVALID SIMULATION CONFIGURATION, SEE ABOVE. ***");
                return;
            }
        }

        running_ = true;
        frc::RobotController::SetTimeSource([]() -> uint64_t {
            return static_cast<uint64_t>(GetTimestamp().value() * 1'000'000.0);
        });
        lastTimestamp_ = static_cast<int64_t>(frc::Timer::GetFPGATimestamp().value() * 1'000'000.0);
        Clear();
        LogTable meta = LogTable(currentStorage_).GetSubtable(IsReplayMode() ? "ReplayMetadata" : "RealMetadata");
        for (const auto& [k, v] : metadata_) meta.Put(k, v);
        if (IsReplayMode()) replaySource_->Start();
        receiverThread_.Start();
        PeriodicBeforeUser();
    }

    void Logger::End() {
        if (!running_) return;
        running_ = false;
        frc::RobotController::SetTimeSource([]() -> uint64_t {
            return frc::RobotController::GetFPGATime();
        });
        if (IsReplayMode()) {
            replaySource_->End();
        }
        receiverThread_.End();
        RadioLogger::Stop();
        LoggedPowerDistribution::Reset();
    }

    void Logger::PeriodicBeforeUser() {
        cycles_++;
        if (!running_) return;

        if (!IsReplayMode()) {
            currentStorage_.timestamp = static_cast<int64_t>(frc::Timer::GetFPGATimestamp().value() * 1'000'000.0);
            return;
        }

        LogTable root(currentStorage_);
        if (!replaySource_->UpdateTable(root)) {
            End();
            return;
        }

        LoggedDriverStation::ReplayFromLog(root.GetSubtable("DriverStation"));
    }

    void Logger::PeriodicAfterUser() {
        PeriodicAfterUser(0, 0);
    }

    void Logger::PeriodicAfterUser(const int64_t userCodeUs, const int64_t periodicBeforeUs) {
        PeriodicAfterUser(userCodeUs, periodicBeforeUs, "");
    }

    void Logger::PeriodicAfterUser(const int64_t userCodeUs, const int64_t periodicBeforeUs,
                                   const std::string_view extraConsoleData) {
        if (!running_) return;
        uint64_t afterStart = frc::RobotController::GetFPGATime();

        LogTable root(currentStorage_);
        if (!HasReplaySource()) {
            LogTable dsTable = root.GetSubtable("DriverStation");
            LoggedDriverStation::SaveToLog(dsTable);
            LoggedPowerDistribution::GetInstance()->SaveToLog(root.GetSubtable("PowerDistribution"));
            LoggedSystemStats::SaveToLog(root.GetSubtable("SystemStats"));
        }
        const uint64_t radioStart = frc::RobotController::GetFPGATime();
        if (!HasReplaySource()) {
            RadioLogger::Periodic(root.GetSubtable("RadioStatus"));
            RecordOutput("Logger/RadioLogMS", (frc::RobotController::GetFPGATime() - radioStart) / 1000.0);
        }
        LogTable loggerTable = root.GetSubtable("Logger");
        loggerTable.Put("Timestamp", currentStorage_.timestamp / 1'000'000.0);
        loggerTable.Put("TimeSinceLastCycle", (currentStorage_.timestamp - lastTimestamp_) / 1'000'000.0);
        loggerTable.Put("CycleCount", static_cast<int64_t>(cycles_));
        lastTimestamp_ = currentStorage_.timestamp;

        uint64_t afterEnd = frc::RobotController::GetFPGATime();
        int64_t periodicAfterUs = static_cast<int64_t>(afterEnd - afterStart);

        LogTable robotTable = root.GetSubtable("LoggedRobot");
        robotTable.Put("UserCodeMS", userCodeUs / 1000.0);
        robotTable.Put("LogPeriodicMS", (periodicBeforeUs + periodicAfterUs) / 1000.0);
        robotTable.Put("FullCycleMS", (periodicBeforeUs + userCodeUs + periodicAfterUs) / 1000.0);

        loggerTable.Put("QueuedCycles", static_cast<int64_t>(receiverThread_.QueueSize()));
        LogStorage snapshot;
        LogTable::Clone(root, snapshot);
        if (!receiverThread_.Enqueue(std::move(snapshot))) {
            FRC_ReportError(frc::err::Error,
                            "[AdvantageKit] Capacity of receiver queue exceeded, data will NOT be logged.");
        }
    }

    void Logger::RecordOutput(const std::string& key, LogValue value) {
        if (!running_) return;

        LogTable outputs = LogTable(currentStorage_).GetSubtable(IsReplayMode() ? "ReplayOutputs" : "RealOutputs");
        outputs.Put(key, std::move(value));
    }

    void Logger::RecordOutput(const std::string& key, const bool value) {
        RecordOutput(key, LogValue{value});
    }

    void Logger::RecordOutput(const std::string& key, const int64_t value) {
        RecordOutput(key, LogValue{value});
    }

    void Logger::RecordOutput(const std::string& key, const float value) {
        RecordOutput(key, LogValue{value});
    }

    void Logger::RecordOutput(const std::string& key, const double value) {
        RecordOutput(key, LogValue{value});
    }

    void Logger::RecordOutput(const std::string& key, const std::string_view value) {
        RecordOutput(key, LogValue{std::string(value)});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const uint8_t> value) {
        RecordOutput(key, LogValue{std::vector<uint8_t>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const bool> value) {
        RecordOutput(key, LogValue{std::vector<bool>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::vector<bool>& value) {
        RecordOutput(key, LogValue{value});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const int> value) {
        RecordOutput(key, LogValue{std::vector<int64_t>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const int64_t> value) {
        RecordOutput(key, LogValue{std::vector<int64_t>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const float> value) {
        RecordOutput(key, LogValue{std::vector<float>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const double> value) {
        RecordOutput(key, LogValue{std::vector<double>(value.begin(), value.end())});
    }

    void Logger::RecordOutput(const std::string& key, const std::span<const std::string> value) {
        RecordOutput(key, LogValue{std::vector<std::string>(value.begin(), value.end())});
    }

    void Logger::RecordMetadata(const std::string& key, const std::string_view value) {
        if (running_) return;
        metadata_[std::string(key)] = std::string(value);
    }

    units::second_t Logger::GetTimestamp() {
        std::scoped_lock lock(mutex_);
        auto time = !running_ || currentStorage_.Empty()
                        ? units::microsecond_t{static_cast<double>(frc::RobotController::GetFPGATime())}
                        : units::microsecond_t{static_cast<double>(currentStorage_.timestamp)};
        return time;
    }

    void Logger::SetReplaySource(LogReplaySource* source) {
        if (running_) return;
        replaySource_ = source;
    }

    bool Logger::HasReplaySource() {
        return IsReplayMode();
    }

    void Logger::DumpCurrentStorage() {
        for (const auto& [key, lv] : currentStorage_.values) {
            std::cout << key << " = ";
            std::visit([]<typename T0>(const T0& v) {
                using T = std::decay_t<T0>;
                if constexpr (std::is_same_v<T, std::vector<double>>
                              || std::is_same_v<T, std::vector<float>>
                              || std::is_same_v<T, std::vector<int64_t>>
                              || std::is_same_v<T, std::vector<uint8_t>>) {
                    std::cout << "[";
                    for (size_t i = 0; i < v.size(); ++i)
                        std::cout << v[i] << (i + 1 < v.size() ? ", " : "");
                    std::cout << "]";
                } else if constexpr (std::is_same_v<T, std::vector<bool>>) {
                    std::cout << "[";
                    for (size_t i = 0; i < v.size(); ++i)
                        std::cout << (v[i] ? "true" : "false") << (i + 1 < v.size() ? ", " : "");
                    std::cout << "]";
                } else if constexpr (std::is_same_v<T, std::vector<std::string>>) {
                    std::cout << "[";
                    for (size_t i = 0; i < v.size(); ++i)
                        std::cout << "\"" << v[i] << "\"" << (i + 1 < v.size() ? ", " : "");
                    std::cout << "]";
                } else {
                    std::cout << v;
                }
            }, lv.value);
            std::cout << "\n";
        }
    }

    void Logger::Clear() {
        currentStorage_.Clear();
        currentStorage_.timestamp = 0;
    }

    const LogStorage& Logger::GetCurrentStorage() {
        return currentStorage_;
    }

    void Logger::AddDataReceiver(LogDataReceiver* receiver) {
        if (running_) {
            receiverThread_.AddDataReceiverAndCatchUp(receiver, LogTable(currentStorage_));
        } else {
            receiverThread_.AddDataReceiver(receiver);
        }
    }

    void Logger::ClearReceivers() {
        receiverThread_.ClearDataReceivers();
    }

    bool Logger::GetReceiverQueueFault() {
        return receiverThread_.HasFault();
    }
}
