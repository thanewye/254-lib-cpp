#include "lib/subsystems/CanCoderIOHardware.h"

#include "lib/util/CANStatusLogger.h"
#include "lib/util/CTREUtil.h"
#include "lib/util/StatusSignalManager.h"
#include <iostream>
#include <cmath>

CanCoderIOHardware::CanCoderIOHardware(CanCoderConfig config)
    : canCoder(config.CANID.GetDeviceNumber(), config.CANID.GetBus())
    , config(config)
    , positionSignal(canCoder.GetAbsolutePosition())
    , velocitySignal(canCoder.GetVelocity()) {
    CTREUtil::ApplyConfiguration(canCoder, config.config);

    signals[0] = &positionSignal;
    signals[1] = &velocitySignal;

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(units::hertz_t(100.0), signals);
    for (int i = 0; i < 2; i++) {
        StatusSignalManager::GetInstance().Register(signals[i]);
    }

    CANStatusLogger::GetInstance().RegisterCANcoder("CANcoder_ID" + std::to_string(config.CANID.GetDeviceNumber()), &canCoder,
            config.CANID.GetDeviceNumber(), config.CANID.GetBus());

    // We now wait for 50 good values here before exiting the contructor.
    double position = 0.0, velocity = 0.0;
    for (int i = 0; i < 50; ++i) {
        ctre::phoenix6::BaseStatusSignal::WaitForAll(units::second_t(10.0), positionSignal, velocitySignal);
        velocity = velocitySignal.GetValueAsDouble();
        position = positionSignal.GetValueAsDouble();
    }
    std::cout << "CANcoder ID " << config.CANID.GetDeviceNumber() << " is ready. Final Position: " << position
            << " Final Velocity: " << velocity << std::endl;
}

void CanCoderIOHardware::UpdateFrequency(double hz) {
    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(units::hertz_t(hz), signals);
}

void CanCoderIOHardware::ReadInputs(CanCoderInputs &inputs) {
    if (std::isnan(inputs.absolutePositionRotations)) {
        ctre::phoenix6::BaseStatusSignal::WaitForAll(units::second_t(10.0), signals);
        goodValues++;
    }
    if (goodValues < 50)
        return;

    inputs.absolutePositionRotations = positionSignal.GetValueAsDouble();
    inputs.velocityRotations = velocitySignal.GetValueAsDouble();
}
