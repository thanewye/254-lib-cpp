#pragma once
#include <string>

class WPILOGConstants final {
public:
    WPILOGConstants() = delete;

    static constexpr const char* extraHeader = "AdvantageKit";
    static const inline std::string entryMetadata = "{\"source\":\"AdvantageKit\"}";
    static const inline std::string entryMetadataUnits =
    "{\"source\":\"AdvantageKit\",\"unit\":\"$UNITSTR\"}";
};
