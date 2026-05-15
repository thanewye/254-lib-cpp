#pragma once

#include <functional>
#include <limits>
#include <string>

#include <ctre/phoenix6/TalonFX.hpp>

#include "lib/drivers/CANDeviceId.h"

template<typename pos_t>
struct ServoMotorSubsystemConfig {
    enum class Motor { KrakenX60, KrakenX44, UNSPECIFIED };

    std::string name = "UNNAMED";
    CANDeviceId talonCANID;
    ctre::phoenix6::configs::TalonFXConfiguration fxConfig = ctre::phoenix6::configs::TalonFXConfiguration();
    double updateFrequencyHz = 50.0;

    /* the ratio from motor to final rotor. <1 is reduction */
    double unitToRotorRatio = 1.0;

    /* effective diameter of final rotor in linear subsystems. leave NaN if subsystem is angular */
    double effectiveRotorDiameter = std::numeric_limits<double>::quiet_NaN();

    pos_t kMinPosition{-std::numeric_limits<double>::infinity()};
    pos_t kMaxPosition{std::numeric_limits<double>::infinity()};
    double momentOfInertia = 0.5;

    /* the ratio from motor to final rotor. <1 is reduction */
    double gearing = 1.0;

    Motor simMotorModel = Motor::UNSPECIFIED;

    bool loadSheddingEnabled = false;
    std::function<bool()> loadSheddingCondition = [] { return false; };
    double loadSheddingSupplyCurrentLimitAmps = 0.0;
};
