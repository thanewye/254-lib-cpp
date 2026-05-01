#pragma once

#include <string_view>

#include <ctre/phoenix6/CANBus.hpp>

struct CANDeviceId {
    CANDeviceId(int deviceNumber, std::string_view bus = "")
        : m_deviceNumber(deviceNumber), m_bus(bus) {}

    int GetDeviceNumber() const { return m_deviceNumber; }
    const ctre::phoenix6::CANBus &GetBus() const { return m_bus; }

    bool operator==(const CANDeviceId &other) const {
        return m_deviceNumber == other.m_deviceNumber &&
               m_bus.GetName() == other.m_bus.GetName();
    }

private:
    int m_deviceNumber;
    ctre::phoenix6::CANBus m_bus;
};
