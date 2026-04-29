#pragma once

#include <string>

#include "LogStorage.h"

namespace akit {
    class LogDataReceiver {
    public:
        const std::string timestampKey = "/Timestamp";

        virtual void Start() {}
        virtual void End() {}
        virtual ~LogDataReceiver() = default;
        virtual void PutTable(const LogStorage& storage) = 0;
    };

}
