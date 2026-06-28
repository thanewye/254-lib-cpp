#include "lib/util/StateMachine.h"

#include <cstdio>

#include "akit/Logger.h"
#include "lib/time/RobotTime.h"

StateMachine::State::State(std::string name, StateMachine* machine)
    : name(std::move(name))
    , machine(machine) {}

void StateMachine::State::AddTransition(StateCondition condition, std::string targetState) {
    transitions.push_back({std::move(condition), std::move(targetState)});
}

LoggedTrigger StateMachine::State::RegisterTrigger(StateCondition condition) {
    return LoggedTrigger([this, condition = std::move(condition)] {
        if (!machine->currentState.has_value() || machine->currentState.value() != name) {
            return false;
        }
        double timeInState = (robot_time::GetTimestamp() - machine->lastStageChangeTime).value();
        return condition(timeInState);
    });
}

LoggedTrigger StateMachine::State::RegisterTrigger(std::function<bool()> condition) {
    return RegisterTrigger([condition = std::move(condition)](double) { return condition(); });
}

LoggedTrigger StateMachine::State::RegisterTrigger() {
    return RegisterTrigger([](double) { return true; });
}

StateMachine::StateMachine(std::string name)
    : name(name)
    , currentStateLogKey("StateMachines/" + name + "/CurrentState")
    , lastStateLogKey("StateMachines/" + name + "/LastState")
    , timeInStateLogKey("StateMachines/" + name + "/TimeInState") {}

StateMachine::State& StateMachine::AddState(const std::string& name) {
    auto it = states.find(name);
    if (it != states.end()) {
        return it->second;
    }
    auto [newIt, inserted] = states.emplace(name, State(name, this));
    return newIt->second;
}

void StateMachine::SetInitialState(const std::string& name) {
    if (states.contains(name)) {
        currentState = name;
        lastState = "";
        lastStageChangeTime = robot_time::GetTimestamp();
    } else {
        std::fprintf(stderr, "StateMachine %s: Attempted to set invalid initial state %s\n", this->name.c_str(), name.c_str());
    }
}

std::optional<std::string> StateMachine::GetCurrentState() const {
    return currentState;
}

void StateMachine::Update() {
    if (!currentState.has_value()) {
        return;
    }

    auto it = states.find(currentState.value());
    if (it != states.end()) {
        double timeInState = (robot_time::GetTimestamp() - lastStageChangeTime).value();
        for (auto& transition : it->second.transitions) {
            if (transition.condition(timeInState)) {
                SetState(transition.targetState);
                break;
            }
        }
    }

    double timestamp = robot_time::GetTimestampSeconds();
    akit::Logger::RecordOutput(currentStateLogKey, currentState.value_or(""));
    akit::Logger::RecordOutput(lastStateLogKey, lastState);
    akit::Logger::RecordOutput(timeInStateLogKey, timestamp - lastStageChangeTime.value());
}

void StateMachine::SetState(const std::string& newState) {
    if (newState != currentState.value_or("") && states.contains(newState)) {
        lastState = currentState.value_or("");
        currentState = newState;
        lastStageChangeTime = robot_time::GetTimestamp();
    }
}