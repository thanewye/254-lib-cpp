#include "LogTable.h"
#include <utility>

namespace akit {

LogTable::LogTable(LogStorage& storage, std::string prefix) 
    : storage_(&storage)
    , prefix_(std::move(prefix)) {}

void LogTable::Put(const std::string& key, LogValue value) const {
    storage_->values[FullKey(key)] = std::move(value);
}

bool LogTable::Get(const std::string_view key, const bool defaultValue) const {
    return GetTyped<bool>(key, defaultValue);
}

double LogTable::Get(const std::string_view key, const double defaultValue) const {
    return GetTyped<double>(key, defaultValue);
}

std::string LogTable::Get(const std::string_view key, std::string defaultValue) const {
    return GetTyped<std::string>(key, std::move(defaultValue));
}

int64_t LogTable::Get(const std::string_view key, const int64_t defaultValue) const {
    return GetTyped<int64_t>(key, defaultValue);
}

std::vector<double> LogTable::Get(std::string_view key, std::vector<double> defaultValue) const {
    return GetTyped<std::vector<double>>(key, std::move(defaultValue));
}

LogTable LogTable::GetSubtable(std::string_view key) const {
    std::string nextPrefix = FullKey(key);
    nextPrefix += "/";
    return LogTable(*storage_, std::move(nextPrefix));
}

int64_t LogTable::GetTimestamp() const {
    return storage_->timestamp;
}

void LogTable::SetTimestamp(const int64_t timestamp) const {
    storage_->timestamp = timestamp;
}

const std::unordered_map<std::string, LogValue>& LogTable::GetAll() const {
    return storage_->values;
}

const std::string& LogTable::GetPrefix() const {
    return prefix_;
}

std::string LogTable::FullKey(const std::string_view key) const {
    std::string fullKey = prefix_;
    fullKey += key;
    return fullKey;
}

void LogTable::Clear() {
    if (prefix_.empty()) {
        storage_->values.clear();
        return;
    }

    std::erase_if(storage_->values, [this](const auto& pair) {
        return pair.first.starts_with(prefix_);
    });
}

} // namespace akit
