#pragma once

#include <limits>

struct CanCoderInputs {
    double absolutePositionRotations = std::numeric_limits<double>::quiet_NaN();
    double velocityRotations = 0.0;
};
