#pragma once

#include <functional>

#include <frc/Errors.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <fmt/format.h>

namespace CTREUtil {

inline constexpr int kMaxRetries = 10;

inline ctre::phoenix::StatusCode TryUntilOk(
        int maxAttempts,
        std::function<ctre::phoenix::StatusCode()> command) {
    ctre::phoenix::StatusCode error = ctre::phoenix::StatusCode::OK;
    for (int i = 0; i < maxAttempts; i++) {
        error = command();
        if (error.IsOK()) return error;
    }
    FRC_ReportWarning(
            "CTREUtil: Command failed after {} attempts. Last error: {}",
            maxAttempts, error.GetName());
    return error;
}

inline ctre::phoenix::StatusCode TryUntilOk(
        const int maxAttempts,
        const std::function<ctre::phoenix::StatusCode()>& function,
        int deviceId) {
    ctre::phoenix::StatusCode statusCode = ctre::phoenix::StatusCode::OK;
    for (int i = 0; i < maxAttempts; ++i) {
        statusCode = function();
        if (statusCode == ctre::phoenix::StatusCode::OK) break;
    }
    if (statusCode != ctre::phoenix::StatusCode::OK) {
        FRC_ReportWarning(
                "Error on CTRE device id {}: {}", deviceId, statusCode.GetName());
    }
    return statusCode;
}

inline ctre::phoenix::StatusCode TryUntilOk(
        const std::function<ctre::phoenix::StatusCode()>& function, const int deviceId) {
    return TryUntilOk(kMaxRetries, function, deviceId);
}


inline ctre::phoenix::StatusCode ApplyConfiguration(
        ctre::phoenix6::hardware::TalonFX& motor,
        const ctre::phoenix6::configs::TalonFXConfiguration& config) {
    return TryUntilOk([&] { return motor.GetConfigurator().Apply(config); },
                                        motor.GetDeviceID());
}

inline ctre::phoenix::StatusCode ApplyConfiguration(
        ctre::phoenix6::hardware::TalonFX& motor,
        const ctre::phoenix6::configs::VoltageConfigs& config) {
    return TryUntilOk([&] { return motor.GetConfigurator().Apply(config); },
                                        motor.GetDeviceID());
}

inline ctre::phoenix::StatusCode ApplyConfigurationNonBlocking(
        ctre::phoenix6::hardware::TalonFX& motor,
        const ctre::phoenix6::configs::VoltageConfigs& config) {
    return motor.GetConfigurator().Apply(config, 0.01_s);
}

inline ctre::phoenix::StatusCode ApplyConfigurationNonBlocking(
        ctre::phoenix6::hardware::TalonFX& motor,
        const ctre::phoenix6::configs::CurrentLimitsConfigs& config) {
    return motor.GetConfigurator().Apply(config, 0.001_s);
}

inline ctre::phoenix::StatusCode ApplyConfiguration(
        ctre::phoenix6::hardware::TalonFX& motor,
        const ctre::phoenix6::configs::HardwareLimitSwitchConfigs& config) {
    return TryUntilOk([&] { return motor.GetConfigurator().Apply(config); },
                                        motor.GetDeviceID());
}

inline ctre::phoenix::StatusCode ApplyConfiguration(
        ctre::phoenix6::hardware::TalonFX& motor,
        const ctre::phoenix6::configs::SoftwareLimitSwitchConfigs& config) {
    return TryUntilOk([&] { return motor.GetConfigurator().Apply(config); },
                                        motor.GetDeviceID());
}

inline ctre::phoenix::StatusCode ApplyConfiguration(
        ctre::phoenix6::hardware::TalonFX& motor,
        const ctre::phoenix6::configs::MotionMagicConfigs& config) {
    return TryUntilOk([&] { return motor.GetConfigurator().Apply(config); },
                                        motor.GetDeviceID());
}

inline ctre::phoenix::StatusCode ApplyConfiguration(
        ctre::phoenix6::hardware::TalonFX& motor,
        const ctre::phoenix6::configs::CurrentLimitsConfigs& config) {
    return TryUntilOk([&] { return motor.GetConfigurator().Apply(config); },
                                        motor.GetDeviceID());
}

inline ctre::phoenix::StatusCode ApplyConfiguration(
        ctre::phoenix6::hardware::CANrange& canrange,
        const ctre::phoenix6::configs::CANrangeConfiguration& config) {
    return TryUntilOk([&] { return canrange.GetConfigurator().Apply(config); },
                                        canrange.GetDeviceID());
}

inline ctre::phoenix::StatusCode ApplyConfiguration(
        ctre::phoenix6::hardware::CANcoder& cancoder,
        const ctre::phoenix6::configs::CANcoderConfiguration& config) {
    return TryUntilOk([&] { return cancoder.GetConfigurator().Apply(config); },
                                        cancoder.GetDeviceID());
}

inline ctre::phoenix::StatusCode ApplyConfiguration(
        ctre::phoenix6::hardware::CANcoder& cancoder,
        const ctre::phoenix6::configs::CANcoderConfiguration& config,
        const units::time::second_t timeout) {
    return TryUntilOk(
            [&] { return cancoder.GetConfigurator().Apply(config, timeout); },
            cancoder.GetDeviceID());
}

inline ctre::phoenix::StatusCode RefreshConfiguration(
        ctre::phoenix6::hardware::TalonFX& motor,
        ctre::phoenix6::configs::TalonFXConfiguration& config) {
    return TryUntilOk([&] { return motor.GetConfigurator().Refresh(config); },
                                        motor.GetDeviceID());
}

inline void ConfigureTalonFX(
        ctre::phoenix6::hardware::TalonFX& talon,
        const ctre::phoenix6::configs::TalonFXConfiguration& config) {
    ApplyConfiguration(talon, config);
}
inline ctre::phoenix6::configs::CANdiConfiguration CreateCustomCandiConfiguration(
        ctre::phoenix6::signals::S1CloseStateValue s1CloseState,
        ctre::phoenix6::signals::S1FloatStateValue s1FloatState,
        ctre::phoenix6::signals::S2CloseStateValue s2CloseState,
        ctre::phoenix6::signals::S2FloatStateValue s2FloatState) {
    ctre::phoenix6::configs::CANdiConfiguration config{};
    config.DigitalInputs.S1CloseState = s1CloseState;
    config.DigitalInputs.S1FloatState = s1FloatState;
    config.DigitalInputs.S2CloseState = s2CloseState;
    config.DigitalInputs.S2FloatState = s2FloatState;
    return config;
}

inline ctre::phoenix6::configs::CANdiConfiguration CreateCandiConfiguration() {
    return CreateCustomCandiConfiguration(
            ctre::phoenix6::signals::S1CloseStateValue::CloseWhenNotHigh,
            ctre::phoenix6::signals::S1FloatStateValue::PullLow,
            ctre::phoenix6::signals::S2CloseStateValue::CloseWhenNotHigh,
            ctre::phoenix6::signals::S2FloatStateValue::PullLow);
}
}    // namespace CTREUtil
