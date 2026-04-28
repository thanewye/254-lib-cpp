#pragma once

#include <string>
#include <string_view>

#include "LoggableInputs.h"
#include "LogStorage.h"

namespace akit {

class Logger {
public:
    static void Start();
    static void End();

    static void ProcessInputs(std::string_view key, LoggableInputs& inputs);
    static void RecordOutput(std::string key, LogValue value);

    static void SetReplayMode(bool replayMode);
    static bool HasReplaySource();
    static void Clear();

    static void PeriodicBeforeUser();
    static void PeriodicAfterUser();

    static const LogStorage& GetCurrentStorage();
    static void DumpCurrentStorage();

private:
    static LogStorage currentStorage_;
    static bool replayMode_;
    static bool running_;
    static int cycles_;
};

}