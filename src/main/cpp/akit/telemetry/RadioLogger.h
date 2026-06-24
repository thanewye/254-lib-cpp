#pragma once

#include <memory>
#include <mutex>
#include <regex>
#include <string>

#include <frc/Notifier.h>

#include "akit/log/LogTable.h"

namespace akit {
    class RadioLogger {
    public:
        static void Periodic(LogTable table);
        static void Stop();

    private:
        static void Start();
        static std::mutex mutex_;
        static bool isConnected_;
        static std::string statusJson_;
        static std::string statusURL_;
        static constexpr int connectTimeoutSecs = 1;
        static constexpr int readTimeoutSecs = 1;
        static constexpr int requestPeriodSecs = 5;
        static const std::regex whitespacePattern;
        static std::unique_ptr<frc::Notifier> notifier_;
    };
} // namespace akit
