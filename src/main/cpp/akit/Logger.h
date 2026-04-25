#pragma once

#include <string>
#include <string_view>

#include "LogTable.h"
#include "LoggableInputs.h"

namespace akit {

class Logger {
public:
    static void ProcessInputs(std::string_view key, LoggableInputs& inputs);
    static void RecordOutput(std::string key, LogValue value);
    static void SetReplayMode(bool replayMode);
    static void Clear();
    static const LogTable& GetCurrentTable() { return currentTable_; }

private:
    static LogTable currentTable_;
    static bool replayMode_;
};

}