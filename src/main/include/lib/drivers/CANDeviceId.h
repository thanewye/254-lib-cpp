#pragma once

#include <ctre/phoenix6/CANBus.hpp>
#include <string_view>

struct CANDeviceId {
  CANDeviceId(int deviceNumber, std::string_view bus = "");

  int GetDeviceNumber() const { return m_deviceNumber; }
  const ctre::phoenix6::CANBus& GetBus() const { return m_bus; }
  bool operator==(const CANDeviceId& other) const;

 private:
  int m_deviceNumber;
  ctre::phoenix6::CANBus m_bus;
};
