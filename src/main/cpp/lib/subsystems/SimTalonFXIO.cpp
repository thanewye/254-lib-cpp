#include "lib/subsystems/SimTalonFXIO.h"

// TODO: add DCMotorSim + TalonFXSimState implementation
// Reference: src/main/java/com/team254/lib/SimTalonFXIO.java

SimTalonFXIO::SimTalonFXIO(const ServoMotorSubsystemConfig& cfg)
    : TalonFXIO(cfg) {}

void SimTalonFXIO::UpdateInputs(MotorInputs& inputs) {
  // TODO: port from Java SimTalonFXIO — update sim state and populate inputs
}
