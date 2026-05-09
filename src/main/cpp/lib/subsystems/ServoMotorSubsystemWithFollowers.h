#pragma once

#include <vector>

#include "lib/subsystems/ServoMotorSubsystem.h"
#include "lib/subsystems/ServoMotorSubsystemWithFollowersConfig.h"

template<IsMotorInputs T, IsMotorIO U>
class ServoMotorSubsystemWithFollowers : public ServoMotorSubsystem<T, U> {
public:
    ServoMotorSubsystemWithFollowers(ServoMotorSubsystemWithFollowersConfig &config,
                                     T leaderInputs, U *leaderIO,
                                     std::vector<T> followerInputs, std::vector<U *> followerIOs);

    void Periodic() override;

    void SetCurrentPositionAsZero() override;

    void SetCurrentPosition(double positionUnits) override;

    double GetCurrentPosition() const override;

    double GetLeaderVelocity() const { return this->inputs.velocityUnitsPerSecond; }
    
    double GetCurrentVelocity() const override;

    double GetStatorCurrentAmps() const override;

    double GetSupplyCurrentAmps() const override;

    double GetAverageStatorCurrentAmps() const override;

    double GetAverageSupplyCurrentAmps() const override;

    frc2::CommandPtr SystemTestCommand(std::string_view testName, double dutyCycle, units::second_t duration);

protected:
    ServoMotorSubsystemWithFollowersConfig leaderConfig;
    std::vector<ServoMotorSubsystemWithFollowersConfig::FollowerConfig> followerConfigs;
    std::vector<T> followerInputs;
    std::vector<U *> followerIOs;
    std::vector<std::string> followerLogKeys;
};

#include "lib/subsystems/ServoMotorSubsystemWithFollowers.ipp"
