#pragma once

#include "akit/Logger.h"
#include "lib/drivers/CANDeviceId.h"
#include "ntcore_c.h"
#include "ntcore_cpp.h"

#include <cstddef>
#include <string>
#include <vector>
#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

class CANStatusLogger {
private:
    CANStatusLogger() = default;

    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    class DeviceStatusInfo {
        friend class CANStatusLogger;
    private:
        std::string name;
        ctre::phoenix6::hardware::TalonFX* talon;
        int deviceID;
        std::string bus;

        std::string loggerKey;
        std::string dashboardKey;
    public:
        ctre::phoenix6::StatusSignal<units::voltage::volt_t> supplyVoltage;
        DeviceStatusInfo(const std::string& name,
            ctre::phoenix6::hardware::TalonFX* talon,
            int deviceID,
            const std::string& bus)
            : name(name)
            , talon(talon)
            , deviceID(deviceID)
            , bus(bus)
            , loggerKey("CANStatus/" + name + "ID" + std::to_string(deviceID))
            , dashboardKey("CAN/" + bus + "/" + name + "ID" + std::to_string(deviceID))
            , supplyVoltage(talon->GetSupplyVoltage()) {
                supplyVoltage.SetUpdateFrequencyForAll(100_Hz);
            }
    };
    std::vector<DeviceStatusInfo> devices;

public:
    static CANStatusLogger& GetInstance() {
        static CANStatusLogger instance;
        return instance;
    }

    void RegisterTalonFX(std::string name, ctre::phoenix6::hardware::TalonFX* talon, int deviceID, ctre::phoenix6::CANBus bus) {
        devices.emplace_back(name, talon, deviceID, std::string(bus.GetName())); 
    }

    void RegisterTalonFX(std::string name, ctre::phoenix6::hardware::TalonFX* talon, CANDeviceId deviceID) {
        devices.emplace_back(name, talon, deviceID.GetDeviceNumber(), std::string(deviceID.GetBus().GetName()));
    }
    
    void RegisterCANcoder(std::string name, ctre::phoenix6::hardware::CANcoder* cancoder, int deviceID, ctre::phoenix6::CANBus bus);

    void InitializeSignalArray() {
        if (signals.size() != devices.size()) {
            signals.clear();
            signals.reserve(devices.size());
            for (DeviceStatusInfo& device : devices) {
                signals.push_back(&device.supplyVoltage);
            }
        }
    }
    void UpdateCanStatus() {
        if (signals.empty()) {
            InitializeSignalArray();
        }
        try {
            ctre::phoenix6::BaseStatusSignal::RefreshAll(signals);
        } catch (std::exception& e) {}

        for (size_t i = 0; i < devices.size(); i++) {
            DeviceStatusInfo& device = devices[i];
            bool isConnected = false;

            if (device.talon != nullptr) {
                isConnected = (device.supplyVoltage.GetStatus() == ctre::phoenix::StatusCode::OK);
            }
            frc::SmartDashboard::PutBoolean(device.dashboardKey, isConnected);
            akit::Logger::RecordOutput(device.loggerKey, isConnected);
        }
    }

};
