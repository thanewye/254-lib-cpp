#pragma once

#include <cmath>
#include <utility>

#include "akit/Logger.h"

template<IsMotorInputs T, IsMotorIO U, IsCanCoderInputs V, IsCanCoderIO W>
ServoMotorSubsystemWithCanCoder<T, U, V, W>::ServoMotorSubsystemWithCanCoder(
    const ServoMotorSubsystemWithCanCoderConfig &config,
    T inputs, U *io, V cancoderInputs, W *cancoderIO)
    : ServoMotorSubsystem<T, U>(config, std::move(inputs), io)
      , canCoderConf(config)
      , cancoderInputs(std::move(cancoderInputs))
      , cancoderIO(cancoderIO) {}

template<IsMotorInputs T, IsMotorIO U, IsCanCoderInputs V, IsCanCoderIO W>
void ServoMotorSubsystemWithCanCoder<T, U, V, W>::Periodic() {
    ServoMotorSubsystem<T, U>::Periodic();
    cancoderIO->ReadInputs(cancoderInputs);
    akit::Logger::ProcessInputs(this->GetName() + "/CanCoder", cancoderInputs);

    if (!canCoderConf.isFusedCancoder && !hasSetOffset &&
        !std::isnan(cancoderInputs.absolutePositionRotations)) {
        this->io->SetCurrentPosition(
            cancoderInputs.absolutePositionRotations * canCoderConf.unitsToCancoderRatio);
        hasSetOffset = true;
    }
}

template<IsMotorInputs T, IsMotorIO U, IsCanCoderInputs V, IsCanCoderIO W>
void ServoMotorSubsystemWithCanCoder<T, U, V, W>::ResetOffset() {
    this->io->SetCurrentPosition(
        cancoderInputs.absolutePositionRotations * canCoderConf.unitsToCancoderRatio);
}
