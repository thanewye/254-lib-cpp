#pragma once

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <units/time.h>

#include "lib/util/LoggedTrigger.h"

class StateMachine {
public:
    using StateCondition = std::function<bool(double)>;

    class State {
    public:
        void AddTransition(StateCondition condition, std::string targetState);
        LoggedTrigger RegisterTrigger(StateCondition condition);
        LoggedTrigger RegisterTrigger(std::function<bool()> condition);
        LoggedTrigger RegisterTrigger();

    private:
        friend class StateMachine;

        struct Transition {
            StateCondition condition;
            std::string targetState;
        };

        State(std::string name, StateMachine* machine);

        std::string name;
        StateMachine* machine;
        std::vector<Transition> transitions;
    };

    explicit StateMachine(std::string name);

    State& AddState(const std::string& name);
    void SetInitialState(const std::string& name);
    [[nodiscard]] std::optional<std::string> GetCurrentState() const;
    void Update();

private:
    std::string name;
    std::string currentStateLogKey;
    std::string lastStateLogKey;
    std::string timeInStateLogKey;

    std::unordered_map<std::string, State> states;
    std::optional<std::string> currentState;
    std::string lastState;
    units::second_t lastStageChangeTime = 0.0_s;

    void SetState(const std::string& newState);
};