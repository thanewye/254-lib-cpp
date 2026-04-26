#include "lib/subsystems/SimTalonFXIO.h"

SimTalonFXIO::SimTalonFXIO(const ServoMotorSubsystemConfig& config)
    : TalonFXIO(config) {}

void SimTalonFXIO::UpdateInputs(MotorInputs& inputs) {
    // TODO: implement
}
