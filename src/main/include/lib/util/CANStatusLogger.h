#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

class CANStatusLogger {
 public:
  static CANStatusLogger& GetInstance();

  void RegisterTalonFX(ctre::phoenix6::hardware::TalonFX* talon);
  void RegisterCANcoder(ctre::phoenix6::hardware::CANcoder* cancoder);
  void UpdateCanStatus();

 private:
  CANStatusLogger() = default;
};
