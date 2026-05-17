#pragma once

#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <magic_enum/magic_enum.hpp>
#include <units/base.h>
#include <units/time.h>
#include <frc/Errors.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>
#include <wpi/struct/Struct.h>

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
        static void RecordOutput(const std::string& key, bool value);
        static void RecordOutput(const std::string& key, int value)       { RecordOutput(key, static_cast<int64_t>(value)); }
        static void RecordOutput(const std::string& key, int64_t value);
        static void RecordOutput(const std::string& key, float value);
        static void RecordOutput(const std::string& key, float value, std::string_view unit)   { RecordOutput(key, LogValue{value, "", std::string(unit)}); }
        static void RecordOutput(const std::string& key, double value);
        static void RecordOutput(const std::string& key, double value, std::string_view unit)  { RecordOutput(key, LogValue{value, "", std::string(unit)}); }
        static void RecordOutput(const std::string& key, const char* value)  { RecordOutput(key, std::string_view(value)); }
        static void RecordOutput(const std::string& key, std::string_view value);

        // 1D array outputs.
        static void RecordOutput(const std::string& key, std::span<const uint8_t> value);
        static void RecordOutput(const std::string& key, std::span<const int> value);
        static void RecordOutput(const std::string& key, std::span<const int64_t> value);
        static void RecordOutput(const std::string& key, std::span<const float> value);
        static void RecordOutput(const std::string& key, std::span<const double> value);
        static void RecordOutput(const std::string& key, std::span<const std::string> value);

        // 2D array outputs — stored as key/length + key/0, key/1, …
        static void RecordOutput(const std::string& key, std::span<const std::vector<uint8_t>> value)  { RecordOutput2D<uint8_t>(key, value); }
        static void RecordOutput(const std::string& key, std::span<const std::vector<int>> value)      { RecordOutput2D<int>(key, value); }
        static void RecordOutput(const std::string& key, std::span<const std::vector<int64_t>> value)  { RecordOutput2D<int64_t>(key, value); }
        static void RecordOutput(const std::string& key, std::span<const std::vector<float>> value)    { RecordOutput2D<float>(key, value); }
        static void RecordOutput(const std::string& key, std::span<const std::vector<double>> value)   { RecordOutput2D<double>(key, value); }
        static void RecordOutput(const std::string& key, std::span<const std::vector<std::string>> value) { RecordOutput2D<std::string>(key, value); }

        // Enum output — stored as the enumerator name string (requires magic_enum).
        template <typename E>
            requires std::is_enum_v<E>
        static void RecordOutput(const std::string& key, E value) {
            RecordOutput(key, std::string_view(magic_enum::enum_name(value)));
        }

        // units::unit_t<U> — stored as double with the unit name as metadata.
        template <typename U>
        static void RecordOutput(const std::string& key, units::unit_t<U> value) {
            RecordOutput(key, value.value(), value.name());
        }

        // frc::Color / frc::Color8Bit — stored as "#RRGGBB" hex string.
        static void RecordOutput(const std::string& key, frc::Color value)     { RecordOutput(key, std::string_view{value.HexString()}); }
        static void RecordOutput(const std::string& key, frc::Color8Bit value) { RecordOutput(key, std::string_view{value.HexString()}); }

        // wpi::StructSerializable<T> — serialized as raw bytes with struct schema registration.
        template <wpi::StructSerializable T>
        static void RecordOutput(const std::string& key, const T& value) {
            if (!running_) return;
            LogTable(currentStorage_).GetSubtable(
                replayMode_ ? "ReplayOutputs" : "RealOutputs").Put(key, value);
        }

        template <wpi::StructSerializable T>
        static void RecordOutput(const std::string& key, const std::vector<T>& values) {
            if (!running_) return;
            LogTable(currentStorage_).GetSubtable(
                replayMode_ ? "ReplayOutputs" : "RealOutputs").Put(key, values);
        }

        static void RecordMetadata(const std::string& key, std::string_view value);

        static void SetReplayMode(bool replayMode);
        static bool HasReplaySource();
        static void Clear();

        static void PeriodicBeforeUser();
        static void PeriodicAfterUser();

        // Runs fn only on every Nth robot cycle — use to downsample expensive outputs.
        template <typename Fn>
        static void RunEveryN(int period, Fn&& fn) {
            if (period < 1) {
                FRC_ReportError(frc::err::Error,
                    "[AdvantageKit] RunEveryN period must be >= 1 (got {})", period);
                return;
            }
            if (cycles_ % period == 0) std::forward<Fn>(fn)();
        }

        static units::second_t GetTimestamp();
        static int GetCycleCount() { return cycles_; }
        static const LogStorage& GetCurrentStorage();
        static void DumpCurrentStorage();

        static void AddDataReceiver(LogDataReceiver* receiver);

        static std::vector<LogDataReceiver*> receivers_;

    private:
        static LogStorage currentStorage_;
        static std::unordered_map<std::string, std::string> metadata_;
        static bool replayMode_;
        static bool running_;
        static int cycles_;
        static int64_t lastTimestamp_;

        template <typename T>
        static void RecordOutput2D(const std::string& key, std::span<const std::vector<T>> value) {
            RecordOutput(key + "/length", static_cast<int64_t>(value.size()));
            for (size_t i = 0; i < value.size(); i++) {
                RecordOutput(key + "/" + std::to_string(i), std::span<const T>(value[i]));
            }
        }
    };
}
