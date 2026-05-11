#pragma once
#include <optional>

class ExponentialMovingAverage {
public:
    explicit ExponentialMovingAverage(double alpha) : alpha(alpha) {}

    double Calculate(double value) {
        if (!oldValue.has_value()) {
            oldValue = value;
            return value;
        }
        double newValue = oldValue.value() + alpha * (value - oldValue.value());
        oldValue = newValue;
        return newValue;
    }

    void Reset() {
        oldValue = std::nullopt;
    }

private:
    double alpha;
    std::optional<double> oldValue;
};
