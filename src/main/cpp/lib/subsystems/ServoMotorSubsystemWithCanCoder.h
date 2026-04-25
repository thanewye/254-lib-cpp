#pragma once

#include "lib/subsystems/ServoMotorSubsystem.h"
#include "lib/subsystems/ServoMotorSubsystemWithCanCoderConfig.h"
#include "lib/subsystems/CanCoderIO.h"
#include "lib/subsystems/CanCoderInputs.h"

class ServoMotorSubsystemWithCanCoder : public ServoMotorSubsystem {
 public:
  ServoMotorSubsystemWithCanCoder(const ServoMotorSubsystemWithCanCoderConfig& config,
                                   MotorIO* leadIO, CanCoderIO* canCoderIO);

  void Periodic() override;
  void ResetOffset();

 protected:
  CanCoderInputs m_canCoderInputs;

 private:
  CanCoderIO* m_canCoderIO;
};
