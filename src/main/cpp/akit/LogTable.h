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

namespace akit {

class LogTable {
public:

    LogTable(LogStorage& storage, std::string prefix = "");

    [[nodiscard]] LogTable GetSubtable(std::string_view name) const;

    // Generic Put — accepts a fully-constructed LogValue.
    void Put(const std::string& key, LogValue value) const;

    // Typed scalar Put overloads.
    void Put(const std::string& key, bool value) const;
    void Put(const std::string& key, int value) const;           // prevents int→float/double ambiguity
    void Put(const std::string& key, int64_t value) const;
    void Put(const std::string& key, float value) const;
    void Put(const std::string& key, float value, std::string_view unit) const;
    void Put(const std::string& key, double value) const;
    void Put(const std::string& key, double value, std::string_view unit) const;
    void Put(const std::string& key, const char* value) const;   // prevents const char*→bool decay
    void Put(const std::string& key, std::string_view value) const;

    // 1D array Put overloads (copy from span into stored vector).
    void Put(const std::string& key, std::span<const uint8_t> value) const;
    void Put(const std::string& key, std::span<const int> value) const;
    void Put(const std::string& key, std::span<const int64_t> value) const;
    void Put(const std::string& key, std::span<const float> value) const;
    void Put(const std::string& key, std::span<const double> value) const;
    void Put(const std::string& key, std::span<const std::string> value) const;

    // 2D array Put overloads — stored as key/length + key/0, key/1, …
    void Put(const std::string& key, std::span<const std::vector<uint8_t>> value) const;
    void Put(const std::string& key, std::span<const std::vector<int>> value) const;
    void Put(const std::string& key, std::span<const std::vector<int64_t>> value) const;
    void Put(const std::string& key, std::span<const std::vector<float>> value) const;
    void Put(const std::string& key, std::span<const std::vector<double>> value) const;
    void Put(const std::string& key, std::span<const std::vector<std::string>> value) const;

    // Enum Put — stored as the enumerator name string (requires magic_enum).
    template <typename E>
        requires std::is_enum_v<E>
    void Put(const std::string& key, E value) const {
        Put(key, std::string_view(magic_enum::enum_name(value)));
    }

    // Scalar Get overloads.
    [[nodiscard]] bool Get(std::string_view key, bool defaultValue) const;
    [[nodiscard]] int64_t Get(std::string_view key, int64_t defaultValue) const;
    [[nodiscard]] float Get(std::string_view key, float defaultValue) const;
    [[nodiscard]] double Get(std::string_view key, double defaultValue) const;
    [[nodiscard]] std::string Get(std::string_view key, std::string defaultValue) const;

    // 1D array Get overloads (default value given as span; returns owned vector).
    [[nodiscard]] std::vector<uint8_t> Get(std::string_view key, std::span<const uint8_t> defaultValue) const;
    [[nodiscard]] std::vector<int> Get(std::string_view key, std::span<const int> defaultValue) const;
    [[nodiscard]] std::vector<int64_t> Get(std::string_view key, std::span<const int64_t> defaultValue) const;
    [[nodiscard]] std::vector<float> Get(std::string_view key, std::span<const float> defaultValue) const;
    [[nodiscard]] std::vector<double> Get(std::string_view key, std::span<const double> defaultValue) const;
    [[nodiscard]] std::vector<std::string> Get(std::string_view key, std::span<const std::string> defaultValue) const;

    // 2D array Get overloads (returns owned vector of vectors).
    [[nodiscard]] std::vector<std::vector<uint8_t>> Get(std::string_view key, std::span<const std::vector<uint8_t>> defaultValue) const;
    [[nodiscard]] std::vector<std::vector<int>> Get(std::string_view key, std::span<const std::vector<int>> defaultValue) const;
    [[nodiscard]] std::vector<std::vector<int64_t>> Get(std::string_view key, std::span<const std::vector<int64_t>> defaultValue) const;
    [[nodiscard]] std::vector<std::vector<float>> Get(std::string_view key, std::span<const std::vector<float>> defaultValue) const;
    [[nodiscard]] std::vector<std::vector<double>> Get(std::string_view key, std::span<const std::vector<double>> defaultValue) const;
    [[nodiscard]] std::vector<std::vector<std::string>> Get(std::string_view key, std::span<const std::vector<std::string>> defaultValue) const;

    // Enum Get — parses the stored name back to the enumerator (requires magic_enum).
    template <typename E>
        requires std::is_enum_v<E>
    [[nodiscard]] E Get(std::string_view key, E defaultValue) const {
        auto name = Get(key, std::string(magic_enum::enum_name(defaultValue)));
        return magic_enum::enum_cast<E>(name).value_or(defaultValue);
    }

    // units::unit_t<U> — stored as double with the unit name as metadata.
    template <typename U>
    void Put(const std::string& key, units::unit_t<U> value) const {
        Put(key, value.value(), value.name());
    }
    template <typename U>
    [[nodiscard]] units::unit_t<U> Get(std::string_view key, units::unit_t<U> defaultValue) const {
        return units::unit_t<U>{Get(key, defaultValue.value())};
    }

    // frc::Color — stored as "#RRGGBB" hex string.
    void Put(const std::string& key, frc::Color value) const {
        Put(key, std::string_view{value.HexString()});
    }
    [[nodiscard]] frc::Color Get(std::string_view key, frc::Color defaultValue) const {
        return frc::Color{Get(key, std::string{defaultValue.HexString()})};
    }

    // frc::Color8Bit — same hex-string pattern, 8-bit components.
    void Put(const std::string& key, frc::Color8Bit value) const {
        Put(key, std::string_view{value.HexString()});
    }
    [[nodiscard]] frc::Color8Bit Get(std::string_view key, frc::Color8Bit defaultValue) const {
        return frc::Color8Bit{Get(key, std::string{defaultValue.HexString()})};
    }

    // wpi::StructSerializable<T> — stored as raw bytes with "struct:TypeName" custom type.
    template <wpi::StructSerializable T>
    void Put(const std::string& key, const T& value) const {
        AddStructSchema<T>();
        std::vector<uint8_t> buf(wpi::Struct<T>::GetSize());
        wpi::PackStruct(std::span{buf}, value);
        Put(key, LogValue{std::move(buf), std::string(wpi::GetStructTypeString<T>())});
    }

    template <wpi::StructSerializable T>
    void Put(const std::string& key, const std::vector<T>& values) const {
        AddStructSchema<T>();
        const size_t elemSize = wpi::Struct<T>::GetSize();
        std::vector<uint8_t> buf(elemSize * values.size());
        for (size_t i = 0; i < values.size(); i++) {
            wpi::PackStruct(std::span{buf}.subspan(i * elemSize, elemSize), values[i]);
        }
        Put(key, LogValue{std::move(buf), std::string(wpi::GetStructTypeString<T>()) + "[]"});
    }

    template <wpi::StructSerializable T>
    [[nodiscard]] T Get(std::string_view key, T defaultValue) const {
        const LogValue* lv = Get(key);
        if (!lv || lv->type != LoggableType::kRaw) return defaultValue;
        if (lv->customTypeStr != wpi::GetStructTypeString<T>()) return defaultValue;
        const auto& raw = std::get<std::vector<uint8_t>>(lv->value);
        if (raw.size() != wpi::Struct<T>::GetSize()) return defaultValue;
        return wpi::UnpackStruct<T>(raw);
    }

    template <wpi::StructSerializable T>
    [[nodiscard]] std::vector<T> Get(std::string_view key, const std::vector<T>& defaultValue = {}) const {
        const LogValue* lv = Get(key);
        if (!lv || lv->type != LoggableType::kRaw) return defaultValue;
        if (lv->customTypeStr != std::string(wpi::GetStructTypeString<T>()) + "[]") return defaultValue;
        const auto& raw = std::get<std::vector<uint8_t>>(lv->value);
        const size_t elemSize = wpi::Struct<T>::GetSize();
        if (elemSize == 0 || raw.size() % elemSize != 0) return defaultValue;
        std::vector<T> result;
        result.reserve(raw.size() / elemSize);
        for (size_t i = 0; i < raw.size(); i += elemSize)
            result.push_back(wpi::UnpackStruct<T>(std::span{raw}.subspan(i, elemSize)));
        return result;
    }

    // Raw LogValue accessor — nullptr if key is absent.
    [[nodiscard]] const LogValue* Get(std::string_view key) const;

    [[nodiscard]] int64_t GetTimestamp() const;
    void SetTimestamp(int64_t timestamp) const;

    // Returns all entries in the storage (full map, by const ref).
    [[nodiscard]] const std::unordered_map<std::string, LogValue>& GetAll() const;

    // Returns entries scoped to this table's prefix when subtableOnly=true,
    // or the full map copy when false.
    [[nodiscard]] std::unordered_map<std::string, LogValue> GetAll(bool subtableOnly) const;

    [[nodiscard]] const std::string& GetPrefix() const;
    [[nodiscard]] int GetDepth() const;

    // Clone into caller-managed storage (caller must keep outStorage alive).
    [[nodiscard]] static LogTable Clone(const LogTable& source, LogStorage& outStorage);

    [[nodiscard]] std::string ToString() const;

    void Clear();

private:
    LogTable(LogStorage& storage, std::string prefix, int depth);

    [[nodiscard]] std::string FullKey(std::string_view key) const;
    [[nodiscard]] bool WriteAllowed(const std::string& fullKey, LoggableType type,
                                    std::string_view customTypeStr = "") const;

    LogStorage* storage_;
    std::string prefix_;
    int depth_ = 0;

    // Registers schema entries for T and all nested struct types it references.
    // Writes directly to storage_->values at the absolute key "/.schema/<typeStr>"
    // so that the table prefix is not applied.
    template <wpi::StructSerializable T>
    void AddStructSchema() const {
        wpi::ForEachStructSchema<T>([this](std::string_view typeStr, std::string_view schema) {
            std::string schemaKey = "/.schema/";
            schemaKey += typeStr;
            if (storage_->values.contains(schemaKey)) return;
            std::vector<uint8_t> bytes(schema.begin(), schema.end());
            storage_->values.emplace(schemaKey,
                LogValue{std::move(bytes), "structschema"});
        });
    }

    template <typename T>
    T GetTyped(const std::string_view key, T defaultValue) const {
        auto it = storage_->values.find(FullKey(key));
        if (it == storage_->values.end()) return defaultValue;

        if (auto* v = std::get_if<T>(&it->second.value)) {
            return *v;
        }

        return defaultValue;
    }

    template <typename T>
    void Put2D(const std::string& key, std::span<const std::vector<T>> value) const {
        Put(key + "/length", static_cast<int64_t>(value.size()));
        for (size_t i = 0; i < value.size(); i++) {
            Put(key + "/" + std::to_string(i), std::span<const T>(value[i]));
        }
    }

    template <typename T>
    std::vector<std::vector<T>> Get2D(std::string_view key,
                                      std::span<const std::vector<T>> defaultValue) const {
        const LogValue* lv = Get(std::string(key) + "/length");
        if (!lv) return {defaultValue.begin(), defaultValue.end()};
        const auto* lenPtr = std::get_if<int64_t>(&lv->value);
        if (!lenPtr) return {defaultValue.begin(), defaultValue.end()};
        std::vector<std::vector<T>> result;
        result.reserve(static_cast<size_t>(*lenPtr));
        for (int64_t i = 0; i < *lenPtr; i++) {
            result.push_back(GetTyped<std::vector<T>>(
                std::string(key) + "/" + std::to_string(i), {}));
        }
        return result;
    }
};

}    // namespace akit
