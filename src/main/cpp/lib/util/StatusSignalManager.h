#pragma once

#include <initializer_list>
#include <vector>

#include <ctre/phoenix6/StatusSignal.hpp>

#include "ctre/phoenix/StatusCodes.h"

class StatusSignalManager {
public:
    static StatusSignalManager& GetInstance();

    void Register(ctre::phoenix6::BaseStatusSignal* signal);
    void Register(std::initializer_list<ctre::phoenix6::BaseStatusSignal*> signalList);
    void RefreshAll();

private:
    StatusSignalManager() = default;
    std::vector<ctre::phoenix6::BaseStatusSignal*> signals;
    ctre::phoenix::StatusCode lastStatus = ctre::phoenix::StatusCode::OK;
};
