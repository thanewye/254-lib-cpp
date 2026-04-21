#include "lib/subsystems/ServoMotorSubsystem.h"

// TODO: add DataLogManager logging for all MotorInputs fields
// Reference: Logger.processInputs() pattern in Java ServoMotorSubsystemWithFollowers

ServoMotorSubsystem::ServoMotorSubsystem(std::string_view name, MotorIO* leadIO,
                                          std::vector<MotorIO*> followerIOs)
    : m_name(name),
      m_leadIO(leadIO),
      m_followerIOs(std::move(followerIOs)),
      m_followerInputs(m_followerIOs.size()) {
  SetName(std::string(name));
}

void ServoMotorSubsystem::Periodic() {
  m_leadIO->UpdateInputs(m_inputs);
  // TODO: log m_inputs fields to DataLogManager
  for (size_t i = 0; i < m_followerIOs.size(); ++i) {
    m_followerIOs[i]->UpdateInputs(m_followerInputs[i]);
  }
}

void ServoMotorSubsystem::SetOpenLoopDutyCycle(double dutyCycle) {
  m_leadIO->SetOpenLoopDutyCycle(dutyCycle);
}
void ServoMotorSubsystem::SetVelocitySetpoint(double unitsPerSecond, int slot) {
  m_leadIO->SetVelocitySetpoint(unitsPerSecond, slot);
}
void ServoMotorSubsystem::SetVelocityMotionMagicSetpoint(double unitsPerSecond, int slot) {
  m_leadIO->SetVelocityMotionMagicSetpoint(unitsPerSecond, slot);
}
void ServoMotorSubsystem::SetMotionMagicSetpoint(double units, int slot) {
  m_leadIO->SetMotionMagicSetpoint(units, slot);
}
void ServoMotorSubsystem::SetVoltageOutput(double voltage) {
  m_leadIO->SetVoltageOutput(voltage);
}
void ServoMotorSubsystem::SetCurrentPositionAsZero() {
  m_leadIO->SetCurrentPositionAsZero();
}
void ServoMotorSubsystem::SetCurrentPosition(double positionUnits) {
  m_leadIO->SetCurrentPosition(positionUnits);
}
void ServoMotorSubsystem::SetSupplyCurrentLimit(double amps) {
  m_leadIO->SetSupplyCurrentLimit(amps);
}
void ServoMotorSubsystem::SetStatorCurrentLimit(double amps) {
  m_leadIO->SetStatorCurrentLimit(amps);
}
