#pragma once

#include <string_view>

#include "akit/LoggableInputs.h"
#include "akit/LogTable.h"

struct MotorInputs : akit::LoggableInputs {
    double velocityUnitsPerSecond = 0.0;
    double unitPosition = 0.0;
    double appliedVolts = 0.0;
    double currentStatorAmps = 0.0;
    double currentSupplyAmps = 0.0;
    double rawRotorPosition = 0.0;
    double motorTempCelsius = 0.0;

    void ToLog(akit::LogTable& table) const override {
        table.Put("velocityUnitsPerSecond", velocityUnitsPerSecond);
        table.Put("unitPosition", unitPosition);
        table.Put("appliedVolts", appliedVolts);
        table.Put("currentStatorAmps", currentStatorAmps);
        table.Put("currentSupplyAmps", currentSupplyAmps);
        table.Put("rawRotorPosition", rawRotorPosition);
        table.Put("motorTempCelsius", motorTempCelsius);
    }

    void FromLog(const akit::LogTable& table) override {
        velocityUnitsPerSecond = table.Get("velocityUnitsPerSecond", 0.0);
        unitPosition = table.Get("unitPosition", 0.0);
        appliedVolts = table.Get("appliedVolts", 0.0);
        currentStatorAmps = table.Get("currentStatorAmps", 0.0);
        currentSupplyAmps = table.Get("currentSupplyAmps", 0.0);
        rawRotorPosition = table.Get("rawRotorPosition", 0.0);
        motorTempCelsius = table.Get("motorTempCelsius", 0.0);
    }
};
