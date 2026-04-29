#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "akit/LogStorage.h"

namespace akit {

class LogTable {
public:

    LogTable(LogStorage& storage, std::string prefix = "");

    [[nodiscard]] LogTable GetSubtable(std::string_view name) const;

    void Put(const std::string &key, LogValue value) const;

    [[nodiscard]] bool Get(std::string_view key, bool defaultValue) const;
    [[nodiscard]] int64_t Get(std::string_view key, int64_t defaultValue) const;
    [[nodiscard]] double Get(std::string_view key, double defaultValue) const;
    [[nodiscard]] std::string Get(std::string_view key, std::string defaultValue) const;
    [[nodiscard]] std::vector<double> Get(std::string_view key, std::vector<double> defaultValue) const;

    [[nodiscard]] int64_t GetTimestamp() const;
    void SetTimestamp(int64_t timestamp) const;

    [[nodiscard]] const std::unordered_map<std::string, LogValue>& GetAll() const;
    [[nodiscard]] const std::string& GetPrefix() const;

    void Clear();

private:
    [[nodiscard]] std::string FullKey(std::string_view key) const;

    LogStorage* storage_;
    std::string prefix_;

    template <typename T>
    T GetTyped(const std::string_view key, T defaultValue) const {
        auto it = storage_->values.find(FullKey(key));
        if (it == storage_->values.end()) return defaultValue;

        if (auto value = std::get_if<T>(&it->second)) {
            return *value;
        }

        return defaultValue;
    }
};

}    // namespace akit
