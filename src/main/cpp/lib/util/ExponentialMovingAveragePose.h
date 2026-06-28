#pragma once

#include <frc/geometry/Pose2d.h>

#include "ExponentialMovingAverage.h"

class ExponentialMovingAveragePose {
public:
    explicit ExponentialMovingAveragePose(double alpha)
        : filterX(ExponentialMovingAverage(alpha))
        , filterY(ExponentialMovingAverage(alpha))
        , filterTheta(ExponentialMovingAverage(alpha)) {}

    void reset() {
        filterX.Reset();
        filterY.Reset();
        filterTheta.Reset();
    }

    frc::Pose2d Calculate(frc::Pose2d value) {
        double newX = filterX.Calculate(static_cast<int>(value.X()));
        double newY = filterY.Calculate(static_cast<int>(value.Y()));
        double newTheta = filterTheta.Calculate(static_cast<int>(value.Rotation().Radians()));

        return frc::Pose2d(units::meter_t{newX}, units::meter_t{newY}, units::radian_t{newTheta});
    }

private:
    ExponentialMovingAverage filterX;
    ExponentialMovingAverage filterY;
    ExponentialMovingAverage filterTheta;
};
