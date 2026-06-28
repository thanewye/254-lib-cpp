#pragma once

#include <memory>
#include <vector>

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/Notifier.h>
#include <frc/simulation/ElevatorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <units/mass.h>

#include "lib/subsystems/ServoMotorSubsystemWithFollowersConfig.h"
#include "lib/subsystems/TalonFXIO.h"

template<typename pos_t> class SimElevator {
public:
    struct SimElevatorConfig {
        double gearing = 1.0;
        double drumRadiusMeters = 0.02;
        double carriageMassKg = 1.0;
        double meterToRotorRatio = 1.0;
        bool simulateGravity = true;
        double startingHeightMeters = 0.0;
    };

    SimElevator(const ServoMotorSubsystemConfig<pos_t>& config, const SimElevatorConfig& simConfig);
    SimElevator(const ServoMotorSubsystemWithFollowersConfig<pos_t>& config, const SimElevatorConfig& simConfig);

    TalonFXIO<pos_t>* GetLeadIO();
    std::vector<TalonFXIO<pos_t>*> GetFollowerIO() const;

    void UpdateSimState();

private:
    class SimElevatorTalonFX : public TalonFXIO<pos_t> {
    public:
        explicit SimElevatorTalonFX(const ServoMotorSubsystemConfig<pos_t>& config);

        ctre::phoenix6::sim::TalonFXSimState& simState;
    };

    static double AddFriction(double motorVoltage, double frictionVoltage);

    ServoMotorSubsystemConfig<pos_t> config;
    SimElevatorConfig simConfig;
    SimElevatorTalonFX leadIO;
    std::vector<std::unique_ptr<SimElevatorTalonFX>> followerIOs;
    std::vector<bool> followerInverted;
    frc::sim::ElevatorSim sim;
    std::unique_ptr<frc::Notifier> simNotifier;
    units::second_t lastUpdateTimestamp{0_s};

    void StartNotifier();
};

#include "lib/subsystems/SimElevator.ipp"
