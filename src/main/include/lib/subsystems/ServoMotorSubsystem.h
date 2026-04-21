#pragma once

#include "lib/subsystems/MotorInputs.h"
#include "lib/subsystems/MotorIO.h"

#include <string>
#include <vector>

#include <frc2/command/SubsystemBase.h>

class ServoMotorSubsystem : public frc2::SubsystemBase {
 public:
  ServoMotorSubsystem(std::string_view name, MotorIO* leadIO,
                      std::vector<MotorIO*> followerIOs = {});

  void Periodic() override;

  virtual double GetCurrentVelocity() const { return m_inputs.velocityUnitsPerSecond; }
  virtual double GetCurrentPosition() const { return m_inputs.unitPosition; }

  void SetOpenLoopDutyCycle(double dutyCycle);
  void SetVelocitySetpoint(double unitsPerSecond, int slot = 0);
  void SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot = 0);
  void SetMotionMagicSetpoint(double units, int slot = 0);
  void SetVoltageOutput(double voltage);
  virtual void SetCurrentPositionAsZero();
  virtual void SetCurrentPosition(double positionUnits);
  void SetSupplyCurrentLimit(double amps);
  void SetStatorCurrentLimit(double amps);

 protected:
  MotorInputs m_inputs;
  std::string m_name;

 private:
  MotorIO* m_leadIO;
  std::vector<MotorIO*> m_followerIOs;
  std::vector<MotorInputs> m_followerInputs;
};
