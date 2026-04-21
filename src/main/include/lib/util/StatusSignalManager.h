#pragma once

#include <ctre/phoenix6/StatusSignal.hpp>
#include <vector>

class StatusSignalManager {
 public:
  static StatusSignalManager& GetInstance();

  void Register(ctre::phoenix6::BaseStatusSignal* signal);
  void RefreshAll();

 private:
  StatusSignalManager() = default;
  std::vector<ctre::phoenix6::BaseStatusSignal*> m_signals;
};
