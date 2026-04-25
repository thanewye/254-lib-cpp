#pragma once

class SqrtProfiledController {
 public:
  SqrtProfiledController();

  void SetConstraints(double maxVelocity, double maxAcceleration);
  void EnableContinuousInput(double minInput, double maxInput);
  void DisableContinuousInput();
  void SetTolerance(double tolerance);
  void SetSetpoint(double setpoint);
  double GetSetpoint() const;
  bool AtSetpoint() const;
  double Calculate(double measurement);
  double Calculate(double measurement, double setpoint);

 private:
  double m_setpoint = 0.0;
  double m_tolerance = 0.0;
};
