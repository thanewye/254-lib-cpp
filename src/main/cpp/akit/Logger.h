#pragma once

#include <string>
#include <string_view>

#include "LogDataReceiver.h"
#include "LogStorage.h"
#include "LoggableInputs.h"

namespace akit {
    class Logger {
    public:
        static void Start();
        static void End();

        template<LoggableAggregate T>
        static void ProcessInputs(std::string_view key, T& inputs) {
            if (!running_) return;
            LogTable currentTable = LogTable(currentStorage_).GetSubtable(key);
            if (replayMode_) FromLog(inputs, currentTable);
            else ToLog(inputs, currentTable);
        }

        static void RecordOutput(const std::string& key, LogValue value);

        static void SetReplayMode(bool replayMode);
        static bool HasReplaySource();
        static void Clear();

        static void PeriodicBeforeUser();
        static void PeriodicAfterUser();

        static const LogStorage& GetCurrentStorage();
        static void DumpCurrentStorage();

        static void AddDataReceiver(LogDataReceiver* receiver);

        static std::vector<LogDataReceiver*> receivers_;

    private:
        static LogStorage currentStorage_;
        static bool replayMode_;
        static bool running_;
        static int cycles_;
    };
}
