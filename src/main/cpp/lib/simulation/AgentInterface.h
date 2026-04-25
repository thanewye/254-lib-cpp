#pragma once

class SimulatedRobotState;

class AgentInterface {
 public:
  virtual ~AgentInterface() = default;
  virtual void Periodic(SimulatedRobotState& state) = 0;
};
