#pragma once

#include "lib/drivers/CANDeviceId.h"

#include <string>
#include <functional>
#include <ctre/phoenix6/TalonFX.hpp>

struct ServoMotorSubsystemConfig {
  enum class Motor { KrakenX60, KrakenX44 };

  std::string name;
  CANDeviceId talonCANID;
  ctre::phoenix6::configs::TalonFXConfiguration fxConfig;
  double updateFrequencyHz = 250.0;
  bool loadSheddingEnabled = false;
  std::function<bool()> loadSheddingCondition;
  double unitToRotorRatio = 1.0;
  double kMinPositionUnits = -1e9;
  double kMaxPositionUnits = 1e9;
  double momentOfInertia = 0.001;
  double gearing = 1.0;
  Motor motor = Motor::KrakenX60;
};
