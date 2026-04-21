#pragma once

#include <memory>

class AgentInterface;

class AgentManager {
 public:
  enum class Mode { FullMatch, Teleop, Auto };

  AgentManager();
  void SimulationPeriodic();
  void Terminate();

 private:
  std::unique_ptr<AgentInterface> m_agent;
};
