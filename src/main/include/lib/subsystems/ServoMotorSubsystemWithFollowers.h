#pragma once

#include "lib/subsystems/ServoMotorSubsystem.h"
#include "lib/subsystems/ServoMotorSubsystemWithFollowersConfig.h"

#include <vector>

class ServoMotorSubsystemWithFollowers : public ServoMotorSubsystem {
 public:
  ServoMotorSubsystemWithFollowers(const ServoMotorSubsystemWithFollowersConfig& config,
                                    MotorIO* leadIO,
                                    std::vector<MotorIO*> followerIOs = {});

  void Periodic() override;

  void SetCurrentPositionAsZero() override;
  void SetCurrentPosition(double positionUnits) override;

  double GetCurrentPosition() const override;
  double GetCurrentVelocity() const override;

  double GetStatorCurrentAmps() const;
  double GetSupplyCurrentAmps() const;
  double GetAverageStatorCurrentAmps() const;
  double GetAverageSupplyCurrentAmps() const;

  frc2::CommandPtr SystemTestCommand();
};
