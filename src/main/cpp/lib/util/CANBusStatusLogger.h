#pragma once

#include <string>

class CANBusStatusLogger {
public:
    explicit CANBusStatusLogger(const std::string &busName);
    void LogStatus();

private:
    std::string m_busName;
};
