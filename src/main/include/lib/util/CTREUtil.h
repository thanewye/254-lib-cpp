#pragma once

#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

namespace CTREUtil {
  ctre::phoenix6::configs::CANdiConfiguration CreateCandiConfiguration();
  void ApplyAndRetry(ctre::phoenix6::hardware::TalonFX& talon,
                     const ctre::phoenix6::configs::TalonFXConfiguration& config);
}
