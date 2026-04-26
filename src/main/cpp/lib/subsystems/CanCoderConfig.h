#pragma once

#include <ctre/phoenix6/CANcoder.hpp>

#include "lib/drivers/CANDeviceId.h"

struct CanCoderConfig {
    CANDeviceId CANID;
    ctre::phoenix6::configs::CANcoderConfiguration config;
};
