#include "lib/util/LogDumper.h"

#include <algorithm>
#include <iomanip>
#include <map>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>

namespace {
    std::string EscapeCsv(std::string value) {
        if (value.find_first_of(",\"\n") == std::string::npos) {
            return value;
        }

        std::string escaped;
        escaped.reserve(value.size() + 2);
        escaped.push_back('"');
        for (const char ch: value) {
            if (ch == '"') {
                escaped.push_back('"');
            }
            escaped.push_back(ch);
        }
        escaped.push_back('"');
        return escaped;
    }

    std::string Join(const std::vector<std::string> &values) {
        std::ostringstream stream;
        for (size_t i = 0; i < values.size(); ++i) {
            if (i != 0) {
                stream << ';';
            }
            stream << values[i];
        }
        return stream.str();
    }

    std::string DecodeValue(const wpi::log::DataLogRecord &record, std::string_view type) {
        if (type == "boolean") {
            bool value = false;
            return record.GetBoolean(&value) ? (value ? "true" : "false") : "";
        }
        if (type == "int64") {
            int64_t value = 0;
            return record.GetInteger(&value) ? std::to_string(value) : "";
        }
        if (type == "float") {
            float value = 0.0f;
            if (!record.GetFloat(&value)) {
                return "";
            }
            std::ostringstream stream;
            stream << value;
            return stream.str();
        }
        if (type == "double") {
            double value = 0.0;
            if (!record.GetDouble(&value)) {
                return "";
            }
            std::ostringstream stream;
            stream << value;
            return stream.str();
        }
        if (type == "string" || type == "json") {
            std::string_view value;
            record.GetString(&value);
            return std::string(value);
        }
        if (type == "boolean[]") {
            std::vector<int> values;
            record.GetBooleanArray(&values);
            std::vector<std::string> parts;
            parts.reserve(values.size());
            for (const int value: values) {
                parts.emplace_back(value ? "true" : "false");
            }
            return Join(parts);
        }
        if (type == "int64[]") {
            std::vector<int64_t> values;
            if (!record.GetIntegerArray(&values)) {
                return "";
            }
            std::vector<std::string> parts;
            parts.reserve(values.size());
            for (const int64_t value: values) {
                parts.emplace_back(std::to_string(value));
            }
            return Join(parts);
        }
        if (type == "float[]") {
            std::vector<float> values;
            if (!record.GetFloatArray(&values)) {
                return "";
            }
            std::vector<std::string> parts;
            parts.reserve(values.size());
            for (const float value: values) {
                std::ostringstream stream;
                stream << value;
                parts.emplace_back(stream.str());
            }
            return Join(parts);
        }
        if (type == "double[]") {
            std::vector<double> values;
            if (!record.GetDoubleArray(&values)) {
                return "";
            }
            std::vector<std::string> parts;
            parts.reserve(values.size());
            for (const double value: values) {
                std::ostringstream stream;
                stream << value;
                parts.emplace_back(stream.str());
            }
            return Join(parts);
        }
        if (type == "string[]") {
            std::vector<std::string_view> values;
            if (!record.GetStringArray(&values)) {
                return "";
            }
            std::vector<std::string> parts;
            parts.reserve(values.size());
            for (const std::string_view value: values) {
                parts.emplace_back(value);
            }
            return Join(parts);
        }

        return "<unsupported:" + std::string(type) + ">";
    }
} // namespace

namespace LogDumper {
    std::vector<Row> ReadRows(
        const std::string &logPath,
        const std::vector<std::string> &keys,
        const int64_t startMicros,
        const int64_t endMicros) {
        auto buffer = wpi::MemoryBuffer::GetFile(logPath);
        if (!buffer) {
            throw std::runtime_error("Failed to open log file: " + logPath);
        }

        wpi::log::DataLogReader reader(std::move(buffer.value()));
        if (!reader.IsValid()) {
            throw std::runtime_error("Invalid log file: " + logPath);
        }

        std::unordered_map<int, std::string> entryNames;
        std::unordered_map<int, std::string> entryTypes;
        std::map<int64_t, std::vector<std::string> > rowsByTimestamp;
        std::unordered_map<std::string, size_t> keyToIndex;

        for (size_t i = 0; i < keys.size(); ++i) {
            keyToIndex.emplace(keys[i], i);
        }

        for (const auto &record: reader) {
            if (record.IsStart()) {
                wpi::log::StartRecordData startData;
                if (record.GetStartData(&startData)) {
                    entryNames.emplace(startData.entry, std::string(startData.name));
                    entryTypes.emplace(startData.entry, std::string(startData.type));
                }
                continue;
            }

            if (record.IsControl() || record.IsSetMetadata()) {
                continue;
            }

            const int64_t timestamp = record.GetTimestamp();
            if (timestamp < startMicros || timestamp > endMicros) {
                continue;
            }

            const auto entryIt = entryNames.find(record.GetEntry());
            if (entryIt == entryNames.end()) {
                continue;
            }

            const auto keyIt = keyToIndex.find(entryIt->second);
            if (keyIt == keyToIndex.end()) {
                continue;
            }

            const auto typeIt = entryTypes.find(record.GetEntry());
            if (typeIt == entryTypes.end()) {
                continue;
            }

            auto &row = rowsByTimestamp[timestamp];
            if (row.empty()) {
                row.resize(keys.size());
            }
            row[keyIt->second] = DecodeValue(record, typeIt->second);
        }

        std::vector<Row> rows;
        rows.reserve(rowsByTimestamp.size());
        for (auto &[timestamp, values]: rowsByTimestamp) {
            rows.push_back(Row{.timestampMicros = timestamp, .values = std::move(values)});
        }
        return rows;
    }

    void WriteCsv(
        const std::vector<Row> &rows,
        const std::vector<std::string> &keys,
        std::ostream &output) {
        output << "Timestamp(s)";
        for (const auto &key: keys) {
            output << ',' << EscapeCsv(key);
        }
        output << '\n';

        for (const auto &row: rows) {
            std::ostringstream timestampStream;
            timestampStream << std::fixed << std::setprecision(6)
                    << static_cast<double>(row.timestampMicros) / 1'000'000.0;
            output << timestampStream.str();

            for (const auto &value: row.values) {
                output << ',' << EscapeCsv(value);
            }
            output << '\n';
        }
    }
}
