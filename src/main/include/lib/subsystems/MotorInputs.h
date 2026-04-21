#pragma once

struct MotorInputs {
    double velocityUnitsPerSecond = 0.0;
    double unitPosition = 0.0;
    double appliedVolts = 0.0;
    double currentStatorAmps = 0.0;
    double currentSupplyAmps = 0.0;
    double rawRotorPosition = 0.0;
    double motorTempCelsius = 0.0;
};
