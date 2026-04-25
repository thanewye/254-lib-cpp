#pragma once

#include "LogTable.h"

namespace akit {

class LoggableInputs {
public:
    virtual ~LoggableInputs() = default;
    virtual void ToLog(LogTable& table) const = 0;
    virtual void FromLog(const LogTable& table) = 0;
};

}