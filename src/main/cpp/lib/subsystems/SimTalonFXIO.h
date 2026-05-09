#pragma once

#include <atomic>
#include <memory>
#include <optional>

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/Notifier.h>
#include <frc/simulation/DCMotorSim.h>

#include "SimCanCoderIO.h"
#include "lib/subsystems/TalonFXIO.h"
#include "lib/subsystems/ServoMotorSubsystemWithCanCoderConfig.h"

class SimTalonFXIO : public TalonFXIO {
public:
    explicit SimTalonFXIO(const ServoMotorSubsystemConfig &config);
    SimTalonFXIO(const ServoMotorSubsystemConfig &config,
                 frc::sim::DCMotorSim sim);
    ~SimTalonFXIO() override = default;

    void ReadInputs(MotorInputs &inputs) override;
    void SetPositionRad(double radians);
    void OverrideRPS(std::optional<double> rps);
    void OverridePosition(std::optional<double> radians);
    void SetInvertVoltage(bool invertVoltage);
    double GetIntendedRPS() const;
    std::function<SimCanCoderIO::SimCanCoderState()> GetSupplierForCancoder(
        const ServoMotorSubsystemWithCanCoderConfig &config);

protected:
    double GetSimRatio() const;
    void UpdateSimState();
    double AddFriction(double motorVoltage, double frictionVoltage) const;

private:
    static frc::DCMotor GetMotorModel(
        ServoMotorSubsystemConfig::Motor motorModel);

    frc::sim::DCMotorSim sim;
    std::unique_ptr<frc::Notifier> simNotifier;
    units::second_t lastUpdateTimestamp{0_s};
    std::optional<double> overrideRPS;
    std::optional<double> overridePositionRad;
    bool invertVoltage = false;

    std::atomic<double> lastRotations{0.0};
    std::atomic<double> lastRPS{0.0};

    const std::string kSimVoltageKey;
    const std::string kSimPositionRadKey;
    const std::string kSimRawRotorPositionKey;
    const std::string kSimVelocityRadSKey;
    const std::string kSetPositionRadKey;
};
