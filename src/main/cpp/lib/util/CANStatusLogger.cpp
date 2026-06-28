#include "lib/util/CANStatusLogger.h"

#include <exception>

#include <frc/smartdashboard/SmartDashboard.h>

#include "akit/Logger.h"

CANStatusLogger::DeviceStatusInfo::DeviceStatusInfo(const std::string& name, ctre::phoenix6::hardware::TalonFX* talon, const int deviceID,
                                                    const std::string& bus)
    : name(name)
    , talon(talon)
    , deviceID(deviceID)
    , bus(bus)
    , loggerKey("CANstatus/" + name + "ID" + std::to_string(deviceID))
    , dashboardKey("CAN/" + bus + "/" + name + "ID" + std::to_string(deviceID))
    , supplyVoltage(talon->GetSupplyVoltage()) {
    supplyVoltage.SetUpdateFrequencyForAll(100_Hz);
}

CANStatusLogger::DeviceStatusInfo::DeviceStatusInfo(const std::string& name, ctre::phoenix6::hardware::CANcoder* cancoder, const int deviceID,
                                                    const std::string& bus)
    : name(name)
    , cancoder(cancoder)
    , deviceID(deviceID)
    , bus(bus)
    , loggerKey("CANstatus/" + name + "ID" + std::to_string(deviceID))
    , dashboardKey("CAN/" + bus + "/" + name + "ID" + std::to_string(deviceID))
    , supplyVoltage(cancoder->GetSupplyVoltage()) {
    supplyVoltage.SetUpdateFrequencyForAll(100_Hz);
}

void CANStatusLogger::RegisterTalonFX(std::string name, ctre::phoenix6::hardware::TalonFX* talon, int deviceID, const ctre::phoenix6::CANBus bus) {
    devices.emplace_back(name, talon, deviceID, std::string(bus.GetName()));
}

void CANStatusLogger::RegisterTalonFX(std::string name, ctre::phoenix6::hardware::TalonFX* talon, CANDeviceId deviceID) {
    devices.emplace_back(name, talon, deviceID.GetDeviceNumber(), std::string(deviceID.GetBus().GetName()));
}

void CANStatusLogger::RegisterCANcoder(std::string name, ctre::phoenix6::hardware::CANcoder* cancoder, int deviceID, const ctre::phoenix6::CANBus bus) {
    (void)name;
    (void)cancoder;
    (void)deviceID;
    (void)bus;
}

void CANStatusLogger::InitializeSignalArray() {
    if (signals.size() != devices.size()) {
        signals.clear();
        signals.reserve(devices.size());
        for (DeviceStatusInfo& device : devices) {
            signals.push_back(&device.supplyVoltage);
        }
    }
}

void CANStatusLogger::UpdateCanStatus() {
    if (signals.empty()) {
        InitializeSignalArray();
    }
    try {
        ctre::phoenix6::BaseStatusSignal::RefreshAll(signals);
    } catch (std::exception& e) {
    }

    for (size_t i = 0; i < devices.size(); i++) {
        DeviceStatusInfo& device = devices[i];
        bool isConnected = false;

        if (device.talon != nullptr) {
            isConnected = device.supplyVoltage.GetStatus() == ctre::phoenix::StatusCode::OK;
        }
        frc::SmartDashboard::PutBoolean(device.dashboardKey, isConnected);
        akit::Logger::RecordOutput(device.loggerKey, isConnected);
    }
}
