#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include <magic_enum/magic_enum.hpp>
#include <units/base.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>
#include <wpi/struct/Struct.h>

#include "akit/LogStorage.h"

using std::vector;
using std::span;
using std::string_view;
using std::string;

namespace akit {
    class LogTable {
    public:
        LogTable(LogStorage& storage, string prefix = "");

        /* --------------------SETTERS (or putters)-------------------- */

        // generic put for fully formed LogValue
        void Put(const string& key, LogValue value) const;

        void Put(const string& key, bool value) const;
        void Put(const string& key, int value) const; // bc number literals are annoying
        void Put(const string& key, int64_t value) const;
        void Put(const string& key, float value) const;
        void Put(const string& key, float value, string_view unit) const;
        void Put(const string& key, double value) const;
        void Put(const string& key, double value, string_view unit) const;

        // because string literals convert to bool for some reason
        void Put(const string& key, const char* value) const;
        void Put(const string& key, string_view value) const;

        // arrays
        void Put(const string& key, span<const uint8_t> value) const;
        void Put(const string& key, span<const int> value) const;
        void Put(const string& key, span<const int64_t> value) const;
        void Put(const string& key, span<const float> value) const;
        void Put(const string& key, span<const double> value) const;
        void Put(const string& key, span<const string> value) const;

        // 2d arrays
        void Put(const string& key, span<const vector<uint8_t>> value) const;
        void Put(const string& key, span<const vector<int>> value) const;
        void Put(const string& key, span<const vector<int64_t>> value) const;
        void Put(const string& key, span<const vector<float>> value) const;
        void Put(const string& key, span<const vector<double>> value) const;
        void Put(const string& key, span<const vector<string>> value) const;

        // enums
        template<typename E>
            requires std::is_enum_v<E>
        void Put(const string& key, E value) const {
            Put(key, string_view(magic_enum::enum_name(value)));
        }

        // wpilib strong units, convenient that they have a name
        template<typename U>
        void Put(const string& key, units::unit_t<U> value) const {
            Put(key, value.value(), value.name());
        }

        // wpilib colors, as hex string
        void Put(const string& key, frc::Color value) const {
            Put(key, string_view{value.HexString()});
        }

        void Put(const string& key, frc::Color8Bit value) const {
            Put(key, string_view{value.HexString()});
        }

        // wpilib struct type using the type string for custom type
        template<wpi::StructSerializable T>
        void Put(const string& key, const T& value) const {
            AddStructSchema<T>();
            vector<uint8_t> buf(wpi::Struct<T>::GetSize());
            wpi::PackStruct(span{buf}, value);
            Put(key, LogValue{std::move(buf), string(wpi::GetStructTypeString<T>())});
        }

        // vector of aforementioned structs
        template<wpi::StructSerializable T>
        void Put(const string& key, const vector<T>& values) const {
            AddStructSchema<T>();
            const size_t elemSize = wpi::Struct<T>::GetSize();
            vector<uint8_t> buf(elemSize * values.size());
            for (size_t i = 0; i < values.size(); i++) {
                wpi::PackStruct(span{buf}.subspan(i * elemSize, elemSize), values[i]);
            }
            Put(key, LogValue{std::move(buf), string(wpi::GetStructTypeString<T>()) + "[]"});
        }

        /* --------------------GETTERS-------------------- */

        // primitives
        [[nodiscard]] bool Get(string_view key, bool defaultValue) const;
        [[nodiscard]] int64_t Get(string_view key, int64_t defaultValue) const;
        [[nodiscard]] float Get(string_view key, float defaultValue) const;
        [[nodiscard]] double Get(string_view key, double defaultValue) const;
        [[nodiscard]] string Get(string_view key, string defaultValue) const;

        // arrays
        [[nodiscard]] vector<uint8_t> Get(string_view key, span<const uint8_t> defaultValue) const;
        [[nodiscard]] vector<int> Get(string_view key, span<const int> defaultValue) const;
        [[nodiscard]] vector<int64_t> Get(string_view key, span<const int64_t> defaultValue) const;
        [[nodiscard]] vector<float> Get(string_view key, span<const float> defaultValue) const;
        [[nodiscard]] vector<double> Get(string_view key, span<const double> defaultValue) const;
        [[nodiscard]] vector<string> Get(string_view key,
                                         span<const string> defaultValue) const;

        // 2d arrays
        [[nodiscard]] vector<vector<uint8_t>> Get(string_view key,
                                                  span<const vector<uint8_t>> defaultValue) const;
        [[nodiscard]] vector<vector<int>> Get(string_view key,
                                              span<const vector<int>> defaultValue) const;
        [[nodiscard]] vector<vector<int64_t>> Get(string_view key,
                                                  span<const vector<int64_t>> defaultValue) const;
        [[nodiscard]] vector<vector<float>> Get(string_view key,
                                                span<const vector<float>> defaultValue) const;
        [[nodiscard]] vector<vector<double>> Get(string_view key,
                                                 span<const vector<double>> defaultValue) const;
        [[nodiscard]] vector<vector<string>> Get(string_view key,
                                                 span<const vector<string>> defaultValue) const;

        template<typename E>
            requires std::is_enum_v<E>
        [[nodiscard]] E Get(string_view key, E defaultValue) const {
            auto name = Get(key, string(magic_enum::enum_name(defaultValue)));
            return magic_enum::enum_cast<E>(name).value_or(defaultValue);
        }

        template<typename U>
        [[nodiscard]] units::unit_t<U> Get(string_view key, units::unit_t<U> defaultValue) const {
            return units::unit_t<U>{Get(key, defaultValue.value())};
        }

        [[nodiscard]] frc::Color Get(string_view key, frc::Color defaultValue) const {
            return frc::Color{Get(key, string{defaultValue.HexString()})};
        }

        [[nodiscard]] frc::Color8Bit Get(string_view key, frc::Color8Bit defaultValue) const {
            return frc::Color8Bit{Get(key, string{defaultValue.HexString()})};
        }

        template<wpi::StructSerializable T>
        [[nodiscard]] T Get(string_view key, T defaultValue) const {
            const LogValue* lv = Get(key);
            if (!lv || lv->type != LoggableType::kRaw) return defaultValue;
            if (lv->customTypeStr != wpi::GetStructTypeString<T>()) return defaultValue;
            const auto& raw = std::get<vector<uint8_t>>(lv->value);
            if (raw.size() != wpi::Struct<T>::GetSize()) return defaultValue;
            return wpi::UnpackStruct<T>(raw);
        }

        template<wpi::StructSerializable T>
        [[nodiscard]] vector<T> Get(string_view key, const vector<T>& defaultValue = {}) const {
            const LogValue* lv = Get(key);
            if (!lv || lv->type != LoggableType::kRaw) return defaultValue;
            if (lv->customTypeStr != string(wpi::GetStructTypeString<T>()) + "[]") return defaultValue;
            const auto& raw = std::get<vector<uint8_t>>(lv->value);
            const size_t elemSize = wpi::Struct<T>::GetSize();
            if (elemSize == 0 || raw.size() % elemSize != 0) return defaultValue;
            vector<T> result;
            result.reserve(raw.size() / elemSize);
            for (size_t i = 0; i < raw.size(); i += elemSize)
                result.push_back(wpi::UnpackStruct<T>(span{raw}.subspan(i, elemSize)));
            return result;
        }

        // raw value accessor w/ no default, returns nullptr (pls don't use this)
        [[nodiscard]] const LogValue* Get(string_view key) const;

        /* --------------------UTIL-------------------- */

        [[nodiscard]] LogTable GetSubtable(string_view name) const;

        [[nodiscard]] int64_t GetTimestamp() const;
        void SetTimestamp(int64_t timestamp) const;

        [[nodiscard]] const std::unordered_map<string, LogValue>& GetAll() const;
        [[nodiscard]] std::unordered_map<string, LogValue> GetAll(bool subtableOnly) const;

        [[nodiscard]] const string& GetPrefix() const;
        [[nodiscard]] int GetDepth() const;

        [[nodiscard]] static LogTable Clone(const LogTable& source, LogStorage& outStorage);

        [[nodiscard]] string ToString() const;

        void Clear();

    private:
        LogTable(LogStorage& storage, string prefix, int depth);

        [[nodiscard]] string FullKey(string_view key) const;
        [[nodiscard]] bool WriteAllowed(const string& fullKey, LoggableType type,
                                        string_view customTypeStr = "") const;

        LogStorage* storage_;
        string prefix_;
        int depth_ = 0;

        template<wpi::StructSerializable T>
        void AddStructSchema() const {
            wpi::ForEachStructSchema<T>([this](string_view typeStr, string_view schema) {
                string schemaKey = "/.schema/";
                schemaKey += typeStr;
                if (storage_->values.contains(schemaKey)) return;
                vector<uint8_t> bytes(schema.begin(), schema.end());
                storage_->values.emplace(schemaKey,
                                         LogValue{std::move(bytes), "structschema"});
            });
        }

        template<typename T>
        T GetTyped(const string_view key, T defaultValue) const {
            auto it = storage_->values.find(FullKey(key));
            if (it == storage_->values.end()) return defaultValue;

            if (auto* v = std::get_if<T>(&it->second.value)) {
                return *v;
            }

            return defaultValue;
        }

        template<typename T>
        void Put2D(const string& key, span<const vector<T>> value) const {
            Put(key + "/length", static_cast<int64_t>(value.size()));
            for (size_t i = 0; i < value.size(); i++) {
                Put(key + "/" + std::to_string(i), span<const T>(value[i]));
            }
        }

        template<typename T>
        vector<vector<T>> Get2D(string_view key,
                                span<const vector<T>> defaultValue) const {
            const LogValue* lv = Get(string(key) + "/length");
            if (!lv) return {defaultValue.begin(), defaultValue.end()};
            const auto* lenPtr = std::get_if<int64_t>(&lv->value);
            if (!lenPtr) return {defaultValue.begin(), defaultValue.end()};
            vector<vector<T>> result;
            result.reserve(static_cast<size_t>(*lenPtr));
            for (int64_t i = 0; i < *lenPtr; i++) {
                result.push_back(GetTyped<vector<T>>(
                    string(key) + "/" + std::to_string(i), {}));
            }
            return result;
        }
    };
}
