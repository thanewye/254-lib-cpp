#pragma once

#include <memory>
#include <string>

class AgentInterface;

class AgentRegistry {
 public:
  static std::unique_ptr<AgentInterface> GetAgent(const std::string& name);
};
