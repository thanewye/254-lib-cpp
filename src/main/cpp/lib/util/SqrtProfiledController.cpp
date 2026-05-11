#include "lib/util/SqrtProfiledController.h"

#include <frc/MathUtil.h>

#include "lib/util/MathHelpers.h"

SqrtProfiledController::SqrtProfiledController(units::radians_per_second_squared_t maxAcceleration,
                                               units::radians_per_second_t maxVelocity, units::radian_t linearThreshold)
    : maxAcceleration(units::math::abs(maxAcceleration))
      , maxVelocity(units::math::abs(maxVelocity))
      , linearThreshold(units::math::abs(linearThreshold)) {
    CalculateKp();
}

void SqrtProfiledController::CalculateKp() {
    if (static_cast<double>(linearThreshold) > 0) {
        kP = sqrt(2.0 * static_cast<double>(maxAcceleration) * static_cast<double>(linearThreshold)) /
             static_cast<double>(linearThreshold);
    } else {
        kP = 0;
    }
}

units::radians_per_second_t SqrtProfiledController::Calculate(units::radian_t measurement) {
    auto error = GetContinuousError(setpoint - measurement);
    auto absError = units::math::abs(error);
    double output;

    if (absError > linearThreshold) {
        // Square Root Controller: sgn(e) * sqrt(2 * a_max * |e|)
        output = MathHelpers::Signum(error.value()) * sqrt(2.0 * maxAcceleration.value() * absError.value());
    } else {
        // Linear Blend: Kp * e
        output = kP * error.value();
    }

    // Clamp the target angular velocity to our chassis max
    return units::radians_per_second_t{std::clamp(output, -maxVelocity.value(), maxVelocity.value())};
}

units::radian_t SqrtProfiledController::GetContinuousError(units::radian_t error) const {
    if (continuous) {
        return frc::InputModulus(error, minimumInput, maximumInput);
    }
    return error;
}
