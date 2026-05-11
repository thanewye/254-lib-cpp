#pragma once

#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/angle.h>
#include <units/math.h>

class SqrtProfiledController {
public:
    SqrtProfiledController(units::radians_per_second_squared_t maxAcceleration,
                           units::radians_per_second_t maxVelocity,
                           units::radian_t linearThreshold);

    void SetConstraints(units::radians_per_second_squared_t maxAcceleration, units::radians_per_second_t maxVelocity) {
        this->maxAcceleration = units::math::abs(maxAcceleration);
        this->maxVelocity = units::math::abs(maxVelocity);
        CalculateKp();
    }

    void EnableContinuousInput(units::radian_t minimumInput, units::radian_t maximumInput) {
        this->continuous = true;
        this->minimumInput = minimumInput;
        this->maximumInput = maximumInput;
    }

    void DisableContinuousInput() {
        this->continuous = false;
    }

    void SetTolerance(units::radian_t positionTolerance) {
        this->positionTolerance = units::math::abs(positionTolerance);
    }

    void SetSetpoint(units::radian_t setpoint) {
        this->setpoint = setpoint;
    }

    units::radian_t GetSetpoint() {
        return this->setpoint;
    }

    [[nodiscard]] bool AtSetpoint(units::radian_t measurement) const {
        auto error = GetContinuousError(setpoint - measurement);
        return units::math::abs(error) <= positionTolerance;
    }

    units::radians_per_second_t Calculate(units::radian_t measurement, units::radian_t setpoint) {
        SetSetpoint(setpoint);
        return Calculate(measurement);
    }

    units::radians_per_second_t Calculate(units::radian_t measurement);

private:
    units::radians_per_second_squared_t maxAcceleration;
    units::radians_per_second_t maxVelocity;
    units::radian_t linearThreshold;
    double kP = 0.0;
    units::radian_t setpoint{0_rad};
    units::radian_t positionTolerance{0.05_rad};
    bool continuous = false;
    units::radian_t minimumInput = 0_rad, maximumInput = 0_rad;

    void CalculateKp();
    [[nodiscard]] units::radian_t GetContinuousError(units::radian_t error) const;
};
