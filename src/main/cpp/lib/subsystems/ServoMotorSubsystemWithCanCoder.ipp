#pragma once

#include <cmath>
#include <utility>

#include "akit/Logger.h"

template<typename pos_t, IsMotorInputs T, IsMotorIO U, IsCanCoderInputs V, IsCanCoderIO W>
ServoMotorSubsystemWithCanCoder<pos_t, T, U, V, W>::ServoMotorSubsystemWithCanCoder(
    const ServoMotorSubsystemWithCanCoderConfig<pos_t>& config,
    T inputs, U* io, V cancoderInputs, W* cancoderIO)
    : ServoMotorSubsystem<pos_t, T, U>(config, std::move(inputs), io)
      , canCoderConf(config)
      , cancoderInputs(std::move(cancoderInputs))
      , cancoderIO(cancoderIO) {}

template<typename pos_t, IsMotorInputs T, IsMotorIO U, IsCanCoderInputs V, IsCanCoderIO W>
void ServoMotorSubsystemWithCanCoder<pos_t, T, U, V, W>::Periodic() {
    ServoMotorSubsystem<pos_t, T, U>::Periodic();
    cancoderIO->ReadInputs(cancoderInputs);
    akit::Logger::ProcessInputs(this->GetName() + "/CanCoder", cancoderInputs);

    if (!canCoderConf.isFusedCancoder && !hasSetOffset &&
        !std::isnan(cancoderInputs.absolutePositionRotations)) {
        this->io->SetCurrentPosition(
            cancoderInputs.absolutePositionRotations * canCoderConf.unitsToCancoderRatio);
        hasSetOffset = true;
    }
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U, IsCanCoderInputs V, IsCanCoderIO W>
void ServoMotorSubsystemWithCanCoder<pos_t, T, U, V, W>::ResetOffset() {
    this->io->SetCurrentPosition(
        cancoderInputs.absolutePositionRotations * canCoderConf.unitsToCancoderRatio);
}
