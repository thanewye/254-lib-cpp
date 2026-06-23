#include "gtest/gtest.h"
#include "akit/telemetry/RadioLogger.h"

namespace {

using akit::RadioLogger;

TEST(RadioLoggerTest, StopIsSafeWhenNeverStarted) {
    RadioLogger::Stop();
}

TEST(RadioLoggerTest, StopIsSafeToCallRepeatedly) {
    RadioLogger::Stop();
    RadioLogger::Stop();
}

}  // namespace
