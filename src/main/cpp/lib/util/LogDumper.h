#pragma once

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

namespace LogDumper {

struct Row {
    int64_t timestampMicros = 0;
    std::vector<std::string> values;
};

std::vector<Row> ReadRows(
    const std::string& logPath,
    const std::vector<std::string>& keys,
    int64_t startMicros = 0,
    int64_t endMicros = INT64_MAX);

void WriteCsv(
    const std::vector<Row>& rows,
    const std::vector<std::string>& keys,
    std::ostream& output);

}
