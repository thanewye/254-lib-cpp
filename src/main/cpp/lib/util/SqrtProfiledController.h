#pragma once

#include <algorithm>
#include <cmath>

#include <frc/MathUtil.h>
#include <units/math.h>
#include <units/time.h>

#include "lib/util/MathHelpers.h"

template<typename PositionUnit> class SqrtProfiledController {
public:
    using position_t = units::unit_t<PositionUnit>;
    using velocity_t = units::unit_t<units::compound_unit<PositionUnit, units::inverse<units::second>>>;
    using acceleration_t = units::unit_t<units::compound_unit<PositionUnit, units::inverse<units::squared<units::second>>>>;

    SqrtProfiledController(acceleration_t maxAcceleration, velocity_t maxVelocity, position_t linearThreshold)
        : maxAcceleration(units::math::abs(maxAcceleration))
        , maxVelocity(units::math::abs(maxVelocity))
        , linearThreshold(units::math::abs(linearThreshold)) {
        CalculateKp();
    }

    void SetConstraints(acceleration_t maxAcceleration, velocity_t maxVelocity) {
        this->maxAcceleration = units::math::abs(maxAcceleration);
        this->maxVelocity = units::math::abs(maxVelocity);
        CalculateKp();
    }

    void EnableContinuousInput(position_t minimumInput, position_t maximumInput) {
        this->continuous = true;
        this->minimumInput = minimumInput;
        this->maximumInput = maximumInput;
    }

    void DisableContinuousInput() { this->continuous = false; }

    void SetTolerance(position_t positionTolerance) { this->positionTolerance = units::math::abs(positionTolerance); }

    void SetSetpoint(position_t setpoint) { this->setpoint = setpoint; }

    [[nodiscard]] position_t GetSetpoint() const { return this->setpoint; }

    [[nodiscard]] bool AtSetpoint(position_t measurement) const {
        auto error = GetContinuousError(setpoint - measurement);
        return units::math::abs(error) <= positionTolerance;
    }

    [[nodiscard]] velocity_t Calculate(position_t measurement, position_t setpoint) {
        SetSetpoint(setpoint);
        return Calculate(measurement);
    }

    [[nodiscard]] velocity_t Calculate(position_t measurement) {
        auto error = GetContinuousError(setpoint - measurement);
        auto absError = units::math::abs(error);
        double output;

        if (absError > linearThreshold) {
            output = math_helpers::Signum(error.value()) * std::sqrt(2.0 * maxAcceleration.value() * absError.value());
        } else {
            output = kP * error.value();
        }

        return velocity_t{std::clamp(output, -maxVelocity.value(), maxVelocity.value())};
    }

private:
    acceleration_t maxAcceleration;
    velocity_t maxVelocity;
    position_t linearThreshold;
    double kP = 0.0;
    position_t setpoint{0};
    position_t positionTolerance{0.05};
    bool continuous = false;
    position_t minimumInput{0}, maximumInput{0};

    void CalculateKp() {
        if (linearThreshold.value() > 0) {
            kP = std::sqrt(2.0 * maxAcceleration.value() * linearThreshold.value()) / linearThreshold.value();
        } else {
            kP = 0;
        }
    }

    [[nodiscard]] position_t GetContinuousError(position_t error) const {
        if (continuous) {
            return frc::InputModulus(error, minimumInput, maximumInput);
        }
        return error;
    }
};

using AngularSqrtProfiledController = SqrtProfiledController<units::radian>;
