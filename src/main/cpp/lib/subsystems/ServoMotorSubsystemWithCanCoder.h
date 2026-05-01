#pragma once

#include "lib/subsystems/CanCoderIO.h"
#include "lib/subsystems/CanCoderInputs.h"
#include "lib/subsystems/ServoMotorSubsystem.h"
#include "lib/subsystems/ServoMotorSubsystemWithCanCoderConfig.h"

template<typename T>
concept IsCanCoderInputs = std::derived_from<T, CanCoderInputs>;

template<typename T>
concept IsCanCoderIO = std::derived_from<T, CanCoderIO>;

template<IsMotorInputs T, IsMotorIO U, IsCanCoderInputs V, IsCanCoderIO W>
class ServoMotorSubsystemWithCanCoder : public ServoMotorSubsystem<T, U> {
public:
    ServoMotorSubsystemWithCanCoder(const ServoMotorSubsystemWithCanCoderConfig &config,
                                    T inputs, U *io, V cancoderInputs, W *cancoderIO);

    void Periodic() override;

    void ResetOffset();

protected:
    ServoMotorSubsystemWithCanCoderConfig canCoderConf;
    bool hasSetOffset = false;
    V cancoderInputs;
    W *cancoderIO;
};

#include "lib/subsystems/ServoMotorSubsystemWithCanCoder.ipp"
