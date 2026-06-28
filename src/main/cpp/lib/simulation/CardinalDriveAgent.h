#pragma once

#include "lib/simulation/AgentInterface.h"

class CardinalDriveAgent : public AgentInterface {
public:
    CardinalDriveAgent();
    void Periodic(SimulatedRobotState& state) override;
};
