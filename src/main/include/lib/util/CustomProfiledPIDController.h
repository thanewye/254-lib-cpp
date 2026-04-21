#pragma once

#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>

class CustomProfiledPIDController {
 public:
  CustomProfiledPIDController(double kP, double kD,
                               frc::TrapezoidProfile<units::meters>::Constraints constraints,
                               double period = 0.02);

  void SetVelocityFF(double ff);
  void SetConstraints(frc::TrapezoidProfile<units::meters>::Constraints constraints);
  void SetGoal(double position, double velocity = 0.0);
  void Reset(double position, double velocity = 0.0);
  double Calculate(double measurement, double setpoint);
  bool AtGoal(double tolerance) const;
  frc::PIDController& GetPIDController();

 private:
  frc::PIDController m_pid;
  double m_velocityFF = 0.0;
};
