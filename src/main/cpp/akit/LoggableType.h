#pragma once

#include <array>
#include <string_view>

namespace akit {
    enum class LoggableType {
        kRaw = 0,
        kBoolean,
        kInteger,
        kFloat,
        kDouble,
        kString,
        kBooleanArray,
        kIntegerArray,
        kFloatArray,
        kDoubleArray,
        kStringArray
    };

    inline constexpr std::array<std::string_view, 11> wpilogTypes = {
        "raw",
        "boolean",
        "int64",
        "float",
        "double",
        "string",
        "boolean[]",
        "int64[]",
        "float[]",
        "double[]",
        "string[]"
    };

    inline constexpr std::array<std::string_view, 11> nt4Types = {
        "raw",
        "boolean",
        "int",
        "float",
        "double",
        "string",
        "boolean[]",
        "int[]",
        "float[]",
        "double[]",
        "string[]"
    };

    inline std::string_view GetWPILOGType(const LoggableType& type) {
        return wpilogTypes[static_cast<int>(type)];
    }

    inline std::string_view GetNT4Type(const LoggableType& type) {
        return nt4Types[static_cast<int>(type)];
    }

    inline LoggableType FromWPILOGType(const std::string_view type) {
        for (int i = 0; i < static_cast<int>(wpilogTypes.size()); i++) {
            if (wpilogTypes[i] == type) return static_cast<LoggableType>(i);
        }
        if (type == "json") return LoggableType::kString;
        return LoggableType::kRaw;
    }

    inline LoggableType FromNT4Type(std::string_view type) {
        for (int i = 0; i < static_cast<int>(nt4Types.size()); i++) {
            if (nt4Types[i] == type) return static_cast<LoggableType>(i);
        }
        if (type == "json") return LoggableType::kString;
        return LoggableType::kRaw;
    }
}