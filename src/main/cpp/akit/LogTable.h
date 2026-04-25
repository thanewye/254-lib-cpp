#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace akit {

using LogValue = std::variant<bool, int64_t, double, std::string, std::vector<double>>;

class LogTable {
public:
    void Put(std::string key, LogValue value) {
        values_[std::move(key)] = std::move(value);
    }

    double Get(std::string_view key, double defaultValue) const {
        return GetTyped<double>(key, defaultValue);
    }

    bool Get(std::string_view key, bool defaultValue) const {
        return GetTyped<bool>(key, defaultValue);
    }

    int64_t Get(std::string_view key, int64_t defaultValue) const {
        return GetTyped<int64_t>(key, defaultValue);
    }

    std::string Get(std::string_view key, std::string defaultValue) const {
        return GetTyped<std::string>(key, std::move(defaultValue));
    }

    std::vector<double> Get(std::string_view key, std::vector<double> defaultValue) const {
        return GetTyped<std::vector<double>>(key, std::move(defaultValue));
    }

private:
    template <typename T>
    T GetTyped(std::string_view key, T defaultValue) const {
        auto it = values_.find(std::string(key));
        if (it == values_.end()) return defaultValue;

        if (auto value = std::get_if<T>(&it->second)) {
            return *value;
        }

        return defaultValue;
    }

    std::unordered_map<std::string, LogValue> values_;
};

}
