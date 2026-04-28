#include "lib/subsystems/SimTalonFXIO.h"

SimTalonFXIO::SimTalonFXIO(const ServoMotorSubsystemConfig& config)
    : TalonFXIO(config) {}

void SimTalonFXIO::ReadInputs(MotorInputs& inputs) {
    // TODO: implement
}
