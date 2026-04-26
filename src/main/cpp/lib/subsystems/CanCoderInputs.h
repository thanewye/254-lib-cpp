#pragma once

#include "akit/LoggableInputs.h"

struct CanCoderInputs : public akit::LoggableInputs {
    double absolutePositionRotations = std::numeric_limits<double>::quiet_NaN();
    double velocityRotations = std::numeric_limits<double>::quiet_NaN();

    void ToLog(akit::LogTable &table) const override {
        table.Put("absolutePositionRotations", absolutePositionRotations);
        table.Put("velocityRotations", velocityRotations);
    }

    void FromLog(const akit::LogTable &table) override {
        absolutePositionRotations = table.Get("absolutePositionRotations", 0.0);
        velocityRotations = table.Get("velocityRotations", 0.0);
    }
};
