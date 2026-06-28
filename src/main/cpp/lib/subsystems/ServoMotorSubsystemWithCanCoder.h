#pragma once

#include "lib/subsystems/CanCoderIO.h"
#include "lib/subsystems/CanCoderInputs.h"
#include "lib/subsystems/ServoMotorSubsystem.h"
#include "lib/subsystems/ServoMotorSubsystemWithCanCoderConfig.h"

template<typename T>
concept IsCanCoderInputs = std::derived_from<T, CanCoderInputs>;

template<typename T>
concept IsCanCoderIO = std::derived_from<T, CanCoderIO>;

template<typename pos_t, IsMotorInputs T, IsMotorIO U, IsCanCoderInputs V, IsCanCoderIO W>
class ServoMotorSubsystemWithCanCoder : public ServoMotorSubsystem<pos_t, T, U> {
public:
    ServoMotorSubsystemWithCanCoder(const ServoMotorSubsystemWithCanCoderConfig<pos_t>& config, T inputs, U* io, V cancoderInputs, W* cancoderIO);

    void Periodic() override;

    void ResetOffset();

protected:
    ServoMotorSubsystemWithCanCoderConfig<pos_t> canCoderConf;
    bool hasSetOffset = false;
    V cancoderInputs;
    W* cancoderIO;
};

#include "lib/subsystems/ServoMotorSubsystemWithCanCoder.ipp"
