#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

namespace akit {
// add more types as necessary
using LogValue = std::variant<bool, int64_t, double, std::string, std::vector<double>>;

/* base storage for all table values for LogTable to search into */
struct LogStorage {
    std::unordered_map<std::string, LogValue> values;
    double timestamp = 0.0;
    void Clear() { values.clear(); }
};

} // namespace akit
