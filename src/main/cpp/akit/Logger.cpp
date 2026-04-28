#include "akit/Logger.h"
#include <frc/Timer.h>
#include <utility>
#include <iostream>

namespace akit {

int Logger::cycles_ = 0;
bool Logger::replayMode_ = false;
bool Logger::running_ = false;
LogStorage Logger::currentStorage_;

void Logger::Start() {
    running_ = true;
    Clear();
}

void Logger::End() {
    running_ = false;
}

void Logger::PeriodicBeforeUser() {
  if (!running_) return
  currentStorage_.Clear();
  currentStorage_.timestamp = frc::Timer::GetFPGATimestamp().value();

  // Later:
  // - real mode: set timestamp from FPGA time
  // - replay mode: read next table from replay source
}

void Logger::PeriodicAfterUser() {
  if (!running_) return;

  // Later:
  // - clone/finalize currentStorage_
  // - send it to data receivers like WPILOGWriter
}

void Logger::ProcessInputs(std::string_view key, LoggableInputs& inputs) {
    if (!running_) return;

    LogTable currentTable = LogTable(currentStorage_).GetSubtable(key);

    if (replayMode_) {
        inputs.FromLog(currentTable);
    } else {
        inputs.ToLog(currentTable);
    }
}

void Logger::RecordOutput(std::string key, LogValue value) {
    if (!running_) return;

    LogTable outputs = LogTable(currentStorage_).GetSubtable(replayMode_ ? "ReplayOutputs" : "RealOutputs");
    outputs.Put(std::move(key), std::move(value));
}

void Logger::SetReplayMode(bool replayMode) { replayMode_ = replayMode; }

bool Logger::HasReplaySource () {
    return replayMode_;
}

void Logger::DumpCurrentStorage() {
    for (const auto& [key, value] : currentStorage_.values) {
        std::cout << key << " = ";
        std::visit([](const auto& v) {
            using T = std::decay_t<decltype(v)>;
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
    currentStorage_.timestamp = 0.0;
}

const LogStorage& Logger::GetCurrentStorage() {
    return currentStorage_;
}
}  // namespace akit
