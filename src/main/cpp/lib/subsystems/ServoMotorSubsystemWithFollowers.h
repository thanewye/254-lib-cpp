#pragma once

#include <vector>

#include "lib/subsystems/ServoMotorSubsystem.h"
#include "lib/subsystems/ServoMotorSubsystemWithFollowersConfig.h"

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
class ServoMotorSubsystemWithFollowers : public ServoMotorSubsystem<pos_t, T, U> {
public:
    using vel_t = typename ServoMotorSubsystem<pos_t, T, U>::vel_t;

    ServoMotorSubsystemWithFollowers(ServoMotorSubsystemWithFollowersConfig<pos_t>& config,
                                     T leaderInputs, U* leaderIO,
                                     std::vector<T> followerInputs, std::vector<U*> followerIOs);

    void Periodic() override;

    void SetCurrentPositionAsZero() override;

    void SetCurrentPosition(pos_t position) override;

    pos_t GetCurrentPosition() const override;

    vel_t GetLeaderVelocity() const { return vel_t{this->inputs.velocityUnitsPerSecond}; }

    vel_t GetCurrentVelocity() const override;

    double GetStatorCurrentAmps() const override;

    double GetSupplyCurrentAmps() const override;

    double GetAverageStatorCurrentAmps() const override;

    double GetAverageSupplyCurrentAmps() const override;

    frc2::CommandPtr SystemTestCommand(std::string_view testName, double dutyCycle, units::second_t duration);

protected:
    ServoMotorSubsystemWithFollowersConfig<pos_t> leaderConfig;
    std::vector<typename ServoMotorSubsystemWithFollowersConfig<pos_t>::FollowerConfig> followerConfigs;
    std::vector<T> followerInputs;
    std::vector<U*> followerIOs;
    std::vector<std::string> followerLogKeys;
};

#include "lib/subsystems/ServoMotorSubsystemWithFollowers.ipp"
