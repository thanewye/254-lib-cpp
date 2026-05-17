#include <algorithm>
#include <cstdint>
#include <iostream>
#include <span>
#include <utility>

#include <frc/Timer.h>

#include "akit/Logger.h"

namespace akit {
    int Logger::cycles_ = 0;
    int64_t Logger::lastTimestamp_ = 0;
    bool Logger::replayMode_ = false;
    bool Logger::running_ = false;
    LogStorage Logger::currentStorage_;
    std::unordered_map<std::string, std::string> Logger::metadata_;
    std::vector<LogDataReceiver*> Logger::receivers_;

    void Logger::Start() {
        running_ = true;
        lastTimestamp_ = static_cast<int64_t>(frc::Timer::GetFPGATimestamp().value() * 1'000'000.0);
        Clear();
        LogTable meta = LogTable(currentStorage_).GetSubtable(
            replayMode_ ? "ReplayMetadata" : "RealMetadata");
        for (const auto& [k, v] : metadata_)
            meta.Put(k, v);
        for (auto* receiver : receivers_) {
            receiver->Start();
        }
    }

    void Logger::End() {
        running_ = false;
        for (auto* receiver : receivers_) {
            receiver->End();
        }
    }

    void Logger::PeriodicBeforeUser() {
        if (!running_) return;
        currentStorage_.Clear();
        currentStorage_.timestamp =
                static_cast<int64_t>(frc::Timer::GetFPGATimestamp().value() * 1'000'000.0);

        // Later:
        // - real mode: set timestamp from FPGA time
        // - replay mode: read next table from replay source
    }

    void Logger::PeriodicAfterUser() {
        if (!running_) return;
        LogTable loggerTable = LogTable(currentStorage_).GetSubtable("Logger");
        loggerTable.Put("Timestamp", currentStorage_.timestamp / 1'000'000.0);
        loggerTable.Put("TimeSinceLastCycle", (currentStorage_.timestamp - lastTimestamp_) / 1'000'000.0);
        loggerTable.Put("CycleCount", static_cast<int64_t>(cycles_));
        lastTimestamp_ = currentStorage_.timestamp;
        LogTable currentTable(currentStorage_);
        for (auto* receiver : receivers_) {
            receiver->PutTable(currentTable);
        }
        cycles_++;
    }

    void Logger::RecordOutput(const std::string& key, LogValue value) {
        if (!running_) return;

        LogTable outputs = LogTable(currentStorage_).GetSubtable(replayMode_ ? "ReplayOutputs" : "RealOutputs");
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
    void Logger::RecordOutput(const std::string& key, const std::span<const int> value) {
        RecordOutput(key, LogValue{std::vector<int>(value.begin(), value.end())});
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
        return units::microsecond_t{static_cast<double>(currentStorage_.timestamp)};
    }

    void Logger::SetReplayMode(const bool replayMode) { replayMode_ = replayMode; }

    bool Logger::HasReplaySource () {
        return replayMode_;
    }

    void Logger::DumpCurrentStorage() {
        for (const auto& [key, lv] : currentStorage_.values) {
            std::cout << key << " = ";
            std::visit([]<typename T0>(const T0& v) {
                using T = std::decay_t<T0>;
                if constexpr (std::is_same_v<T, std::vector<double>>
                           || std::is_same_v<T, std::vector<float>>
                           || std::is_same_v<T, std::vector<int64_t>>
                           || std::is_same_v<T, std::vector<uint8_t>>
                           || std::is_same_v<T, std::vector<int>>) {
                    std::cout << "[";
                    for (size_t i = 0; i < v.size(); ++i)
                        std::cout << v[i] << (i + 1 < v.size() ? ", " : "");
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
    void Logger::AddDataReceiver(LogDataReceiver *receiver) {
        if (std::ranges::find(receivers_, receiver) != receivers_.end()) return;
        receivers_.push_back(receiver);
    }
}
