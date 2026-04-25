#include "akit/Logger.h"

namespace akit {

LogTable Logger::currentTable_;
bool Logger::replayMode_ = false;

void Logger::ProcessInputs(std::string_view key, LoggableInputs& inputs) {
    (void) key;
    if (replayMode_) {
        inputs.FromLog(currentTable_);
    } else {
        inputs.ToLog(currentTable_);
    }
}

void Logger::RecordOutput(std::string key, LogValue value) {
    currentTable_.Put(std::move(key), std::move(value));
}

void Logger::SetReplayMode(bool replayMode) { replayMode_ = replayMode; }

void Logger::Clear() { currentTable_ = LogTable{}; }
}  // namespace akit