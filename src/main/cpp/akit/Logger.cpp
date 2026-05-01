#include <algorithm>
#include <cstdint>
#include <iostream>
#include <utility>

#include <frc/Timer.h>

#include "akit/Logger.h"

namespace akit {
    int Logger::cycles_ = 0;
    bool Logger::replayMode_ = false;
    bool Logger::running_ = false;
    LogStorage Logger::currentStorage_;
    std::vector<LogDataReceiver*> Logger::receivers_;

    void Logger::Start() {
        running_ = true;
        Clear();
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
        LogTable currentTable(currentStorage_);
        for (auto* receiver : receivers_) {
            receiver->PutTable(currentTable);
        }
        cycles_++;
    }

    void Logger::ProcessInputs(const std::string_view key, LoggableInputs& inputs) {
        if (!running_) return;

        LogTable currentTable = LogTable(currentStorage_).GetSubtable(key);

        if (replayMode_) {
            inputs.FromLog(currentTable);
        } else {
            inputs.ToLog(currentTable);
        }
    }

    void Logger::RecordOutput(const std::string& key, LogValue value) {
        if (!running_) return;

        LogTable outputs = LogTable(currentStorage_).GetSubtable(replayMode_ ? "ReplayOutputs" : "RealOutputs");
        outputs.Put(key, std::move(value));
    }

    void Logger::SetReplayMode(const bool replayMode) { replayMode_ = replayMode; }

    bool Logger::HasReplaySource () {
        return replayMode_;
    }

    void Logger::DumpCurrentStorage() {
        for (const auto& [key, value] : currentStorage_.values) {
            std::cout << key << " = ";
            std::visit([]<typename T0>(const T0& v) {
                using T = std::decay_t<T0>;
                if constexpr (std::is_same_v<T, std::vector<double>>) {
                    std::cout << "[";
                    for (size_t i = 0; i < v.size(); ++i) {
                        std::cout << v[i] << (i + 1 < v.size() ? ", " : "");
                    }
                    std::cout << "]";
                } else {
                    std::cout << v;
                }
            }, value);
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
