#pragma once

#include "lib/simulation/AgentInterface.h"

class PickUpAndShootAgent : public AgentInterface {
public:
    enum class State { DrivingToFuel, PickingUp, DrivingToShoot, Shooting };

    PickUpAndShootAgent();
    void Periodic(SimulatedRobotState& state) override;

private:
    State m_state = State::DrivingToFuel;
};
