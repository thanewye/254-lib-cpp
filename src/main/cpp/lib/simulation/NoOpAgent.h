#pragma once

#include "lib/simulation/AgentInterface.h"

class NoOpAgent : public AgentInterface {
public:
    NoOpAgent();
    void Periodic(SimulatedRobotState& state) override;
};
