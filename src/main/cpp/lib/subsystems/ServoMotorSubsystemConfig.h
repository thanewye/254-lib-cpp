#pragma once

#include <functional>
#include <string>

#include <ctre/phoenix6/TalonFX.hpp>

#include "lib/drivers/CANDeviceId.h"

struct ServoMotorSubsystemConfig {
    enum class Motor { KrakenX60, KrakenX44, UNSPECIFIED };

    std::string name = "UNNAMED";
    CANDeviceId talonCANID;
    ctre::phoenix6::configs::TalonFXConfiguration fxConfig = ctre::phoenix6::configs::TalonFXConfiguration();
    double updateFrequencyHz = 50.0;

    double unitToRotorRatio = 1.0;
    double kMinPositionUnits = -std::numeric_limits<double>::infinity();
    double kMaxPositionUnits = std::numeric_limits<double>::infinity();
    double momentOfInertia = 0.5;
    double gearing = 1.0;
    Motor simMotorModel = Motor::UNSPECIFIED;

    bool loadSheddingEnabled = false;
    std::function<bool()> loadSheddingCondition = [] { return false; };
    double loadSheddingSupplyCurrentLimitAmps = 0.0;
};
