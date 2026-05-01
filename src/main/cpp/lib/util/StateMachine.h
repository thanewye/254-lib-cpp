#pragma once

#include <functional>
#include <string>
#include <vector>

class StateMachine {
 public:
  struct State {
    std::string name;
    std::function<void()> onEnter;
    std::function<void()> onExit;
    std::function<void()> periodic;
  };

  using StateCondition = std::function<bool()>;

  void AddState(State state);
  void SetInitialState(const std::string& name);
  const std::string& GetCurrentState() const;
  void Update();

 private:
  std::string m_currentState;
  std::vector<State> m_states;
};
