#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <frc/smartdashboard/SmartDashboard.h>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include "akit/Logger.h"
#include "lib/drivers/CANDeviceId.h"

class CANStatusLogger {
    CANStatusLogger() = default;

    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    class DeviceStatusInfo {
        friend class CANStatusLogger;
        std::string name;
        ctre::phoenix6::hardware::TalonFX* talon = nullptr;
        ctre::phoenix6::hardware::CANcoder* cancoder = nullptr;
        int deviceID;
        std::string bus;

        std::string loggerKey;
        std::string dashboardKey;
    public:
        ctre::phoenix6::StatusSignal<units::voltage::volt_t> supplyVoltage;
        DeviceStatusInfo(const std::string& name,
            ctre::phoenix6::hardware::TalonFX* talon,
            const int deviceID,
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

        DeviceStatusInfo(const std::string& name,
            ctre::phoenix6::hardware::CANcoder* cancoder,
            const int deviceID,
            const std::string& bus)
            : name(name)
            , cancoder(cancoder)
            , deviceID(deviceID)
            , bus(bus)
            , loggerKey("CANStatus/" + name + "ID" + std::to_string(deviceID))
            , dashboardKey("CAN/" + bus + "/" + name + "ID" + std::to_string(deviceID))
            , supplyVoltage(cancoder->GetSupplyVoltage()) {
                supplyVoltage.SetUpdateFrequencyForAll(100_Hz);
            }
    };
    std::vector<DeviceStatusInfo> devices;

public:
    static CANStatusLogger& GetInstance() {
        static CANStatusLogger instance;
        return instance;
    }

    void RegisterTalonFX(std::string name, ctre::phoenix6::hardware::TalonFX* talon, int deviceID, const ctre::phoenix6::CANBus bus) {
        devices.emplace_back(name, talon, deviceID, std::string(bus.GetName())); 
    }

    void RegisterTalonFX(std::string name, ctre::phoenix6::hardware::TalonFX* talon, CANDeviceId deviceID) {
        devices.emplace_back(name, talon, deviceID.GetDeviceNumber(), std::string(deviceID.GetBus().GetName()));
    }
    
    void RegisterCANcoder(std::string name, ctre::phoenix6::hardware::CANcoder* cancoder, int deviceID, const ctre::phoenix6::CANBus bus) {
        devices.emplace_back(name, cancoder, deviceID, std::string(bus.GetName()));
    }

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

            if (device.talon != nullptr || device.cancoder != nullptr) {
                isConnected = device.supplyVoltage.GetStatus() == ctre::phoenix::StatusCode::OK;
            }
            frc::SmartDashboard::PutBoolean(device.dashboardKey, isConnected);
            akit::Logger::RecordOutput(device.loggerKey, isConnected);
        }
    }

};
