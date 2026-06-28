#pragma once

#include <string>
#include <vector>

#include <ctre/phoenix6/CANBus.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

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

        DeviceStatusInfo(const std::string& name, ctre::phoenix6::hardware::TalonFX* talon, const int deviceID, const std::string& bus);

        DeviceStatusInfo(const std::string& name, ctre::phoenix6::hardware::CANcoder* cancoder, const int deviceID, const std::string& bus);
    };

    std::vector<DeviceStatusInfo> devices;

public:
    static CANStatusLogger& GetInstance() {
        static CANStatusLogger instance;
        return instance;
    }

    void RegisterTalonFX(std::string name, ctre::phoenix6::hardware::TalonFX* talon, int deviceID, const ctre::phoenix6::CANBus bus);
    void RegisterTalonFX(std::string name, ctre::phoenix6::hardware::TalonFX* talon, CANDeviceId deviceID);
    void RegisterCANcoder(std::string name, ctre::phoenix6::hardware::CANcoder* cancoder, int deviceID, const ctre::phoenix6::CANBus bus);
    void InitializeSignalArray();
    void UpdateCanStatus();
};
