#include "lib/util/ListKeys.h"

#include <set>
#include <stdexcept>

#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>

namespace list_keys {
    std::vector<std::string> Read(const std::string& logPath) {
        auto buffer = wpi::MemoryBuffer::GetFile(logPath);
        if (!buffer) {
            throw std::runtime_error("Failed to open log file: " + logPath);
        }

        wpi::log::DataLogReader reader(std::move(buffer.value()));
        if (!reader.IsValid()) {
            throw std::runtime_error("Invalid log file: " + logPath);
        }

        std::set<std::string> orderedKeys;
        for (const auto& record : reader) {
            if (!record.IsStart()) {
                continue;
            }

            wpi::log::StartRecordData startData;
            if (record.GetStartData(&startData)) {
                orderedKeys.emplace(startData.name);
            }
        }

        return {orderedKeys.begin(), orderedKeys.end()};
    }
} // namespace list_keys
