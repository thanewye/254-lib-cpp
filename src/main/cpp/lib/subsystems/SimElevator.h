#pragma once

#include <memory>
#include <vector>

#include <ctre/phoenix6/sim/TalonFXSimState.hpp>
#include <frc/system/plant/DCMotor.h>
#include <frc/Notifier.h>
#include <frc/simulation/ElevatorSim.h>
#include <units/mass.h>

#include "lib/subsystems/TalonFXIO.h"
#include "lib/subsystems/ServoMotorSubsystemWithFollowersConfig.h"

class SimElevator {
public:
    struct SimElevatorConfig {
        double gearing = 1.0;
        double drumRadiusMeters = 0.02;
        double carriageMassKg = 1.0;
        double meterToRotorRatio = 1.0;
        bool simulateGravity = true;
        double startingHeightMeters = 0.0;
    };

    SimElevator(const ServoMotorSubsystemConfig &config,
                const SimElevatorConfig &simConfig);
    SimElevator(const ServoMotorSubsystemWithFollowersConfig &config,
                const SimElevatorConfig &simConfig);

    TalonFXIO *GetLeadIO();
    std::vector<TalonFXIO *> GetFollowerIO() const;

    void UpdateSimState();

private:
    class SimElevatorTalonFX : public TalonFXIO {
    public:
        explicit SimElevatorTalonFX(const ServoMotorSubsystemConfig &config);

        ctre::phoenix6::sim::TalonFXSimState &simState;
    };

    static double AddFriction(double motorVoltage, double frictionVoltage);

    ServoMotorSubsystemConfig config;
    SimElevatorConfig simConfig;
    SimElevatorTalonFX leadIO;
    std::vector<std::unique_ptr<SimElevatorTalonFX> > followerIOs;
    std::vector<bool> followerInverted;
    frc::sim::ElevatorSim sim;
    std::unique_ptr<frc::Notifier> simNotifier;
    units::second_t lastUpdateTimestamp{0_s};

    void StartNotifier();
};
