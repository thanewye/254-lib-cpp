#pragma once

#include <string>
#include <ctre/phoenix6/CANBus.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

class CANBusStatusLogger {
public:
    explicit CANBusStatusLogger(const std::string& name)
        : bus(name) {}

    void LogStatus() {
        auto status = bus.GetStatus();
        // logger here once it's supported
        frc::SmartDashboard::PutString("CANBusStatus/" + static_cast<std::string>(bus.GetName()),
                                       status.Status.GetName());
    }

private:
    ctre::phoenix6::CANBus bus;
};
