#include <cmath>
#include <string>
#include <vector>

#include <frc/system/plant/LinearSystemId.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/simulation/SimHooks.h>
#include <units/time.h>

#define private public
#define protected public
#include "gtest/gtest.h"
#include "lib/subsystems/MotorInputs.h"
#include "lib/subsystems/SimElevator.h"
#include "lib/subsystems/SimTalonFXIO.h"
#include "lib/subsystems/TalonFXIO.h"
#undef protected
#undef private
#include "lib/subsystems/ServoMotorSubsystemWithCanCoderConfig.h"
#include "lib/subsystems/ServoMotorSubsystemWithFollowersConfig.h"
#include "lib/util/StatusSignalManager.h"

namespace {

using frc::sim::PauseTiming;
using frc::sim::RestartTiming;
using frc::sim::ResumeTiming;
using frc::sim::StepTiming;

struct CanCoderStateSnapshot {
    double positionRotations;
    double velocityRotations;
};

class SimSubsystemTest : public ::testing::Test {
protected:
    void SetUp() override {
        RestartTiming();
        PauseTiming();
    }

    void TearDown() override {
        ResumeTiming();
    }
};

ServoMotorSubsystemConfig MakeMotorConfig(int deviceId,
                                         const std::string& name) {
    ServoMotorSubsystemConfig config{.talonCANID = CANDeviceId(deviceId)};
    config.name = name;
    config.simMotorModel = ServoMotorSubsystemConfig::Motor::KrakenX60;
    config.gearing = 1.0;
    config.unitToRotorRatio = 1.0;
    config.momentOfInertia = 0.01;
    return config;
}

ServoMotorSubsystemWithCanCoderConfig MakeCanCoderConfig() {
    ServoMotorSubsystemWithCanCoderConfig config{
        MakeMotorConfig(300, "sim-cancoder-host"),
        CanCoderConfig{CANDeviceId(301)}
    };
    config.gearing = 1.0;
    config.canCodergearing = 1.0;
    return config;
}

CanCoderStateSnapshot RunTalonStep(units::second_t preAdvance,
                                   int deviceId,
                                   double gearing = 1.0,
                                   double canCoderGearing = 1.0) {
    if (preAdvance > 0_s) {
        StepTiming(preAdvance);
    }

    frc::sim::DCMotorSim motorSim(
        frc::LinearSystemId::DCMotorSystem(
            frc::DCMotor::KrakenX60(1),
            units::kilogram_square_meter_t{0.01},
            1.0),
        frc::DCMotor::KrakenX60(1),
        {0.0, 0.0});
    motorSim.SetState(0_rad, 12_rad_per_s);

    auto config = MakeMotorConfig(deviceId, "sim-talon-" + std::to_string(deviceId));
    config.gearing = gearing;

    auto* simTalon = new SimTalonFXIO(
        config,
        std::move(motorSim));
    auto canCoderConfig = MakeCanCoderConfig();
    canCoderConfig.gearing = gearing;
    canCoderConfig.canCodergearing = canCoderGearing;
    auto supplier = simTalon->GetSupplierForCancoder(canCoderConfig);

    StepTiming(5_ms);

    const auto state = supplier();
    return CanCoderStateSnapshot{
        .positionRotations = state.positionRotations,
        .velocityRotations = state.velocityRotations,
    };
}

std::vector<MotorInputs> RunElevatorStep(units::second_t preAdvance) {
    if (preAdvance > 0_s) {
        StepTiming(preAdvance);
    }

    ServoMotorSubsystemWithFollowersConfig config{
        MakeMotorConfig(200, "sim-elevator")
    };
    config.gearing = 1.0;
    config.unitToRotorRatio = 1.0;
    config.kMinPositionUnits = -10.0;
    config.kMaxPositionUnits = 10.0;
    config.followers = {
        {.config = MakeMotorConfig(201, "follower-aligned"), .inverted = false},
        {.config = MakeMotorConfig(202, "follower-opposed"), .inverted = true},
    };

    SimElevator::SimElevatorConfig simConfig;
    simConfig.gearing = 1.0;
    simConfig.meterToRotorRatio = 1.0;
    simConfig.drumRadiusMeters = 0.05;
    simConfig.carriageMassKg = 4.0;
    simConfig.startingHeightMeters = 0.5;

    auto* elevator = new SimElevator(config, simConfig);
    TalonFXIO* lead = elevator->GetLeadIO();
    StepTiming(5_ms);
    StatusSignalManager::GetInstance().RefreshAll();

    std::vector<MotorInputs> inputs;
    MotorInputs leadInputs;
    lead->ReadInputs(leadInputs);
    inputs.push_back(leadInputs);

    for (TalonFXIO* follower : elevator->GetFollowerIO()) {
        MotorInputs followerInputs;
        follower->ReadInputs(followerInputs);
        inputs.push_back(followerInputs);
    }

    return inputs;
}

double RunTalonVelocityAfterVoltageStep(double gearing,
                                        units::second_t duration) {
    auto config = MakeMotorConfig(150, "sim-talon-gear-test");
    config.gearing = gearing;

    auto* simTalon = new SimTalonFXIO(config);
    simTalon->talon.GetSimState().SetSupplyVoltage(12_V);
    simTalon->SetVoltageOutput(12.0);

    StepTiming(duration);
    StatusSignalManager::GetInstance().RefreshAll();

    MotorInputs inputs;
    simTalon->ReadInputs(inputs);
    return inputs.velocityUnitsPerSecond;
}

double RunElevatorVelocityAfterVoltageStep(double gearing,
                                           units::second_t duration) {
    auto config = MakeMotorConfig(250, "sim-elevator-gear-test");
    config.kMinPositionUnits = -10.0;
    config.kMaxPositionUnits = 10.0;

    SimElevator::SimElevatorConfig simConfig;
    simConfig.gearing = gearing;
    simConfig.meterToRotorRatio = 1.0;
    simConfig.drumRadiusMeters = 0.05;
    simConfig.carriageMassKg = 4.0;

    auto* elevator = new SimElevator(config, simConfig);
    auto* lead = elevator->GetLeadIO();
    lead->talon.GetSimState().SetSupplyVoltage(12_V);
    lead->SetVoltageOutput(12.0);

    StepTiming(duration);
    StatusSignalManager::GetInstance().RefreshAll();

    MotorInputs inputs;
    lead->ReadInputs(inputs);
    return inputs.velocityUnitsPerSecond;
}

TEST_F(SimSubsystemTest, TalonStartupDeltaDependsOnFpgaUptime) {
    const auto baseline = RunTalonStep(0_s, 101);

    RestartTiming();
    PauseTiming();

    const auto delayed = RunTalonStep(2_s, 102);

    EXPECT_GT(std::abs(delayed.positionRotations - baseline.positionRotations), 1e-6);
    EXPECT_GT(std::abs(delayed.velocityRotations - baseline.velocityRotations), 1e-6);
}

TEST_F(SimSubsystemTest, TalonCancoderUsesCancoderRatioDirection) {
    auto config = MakeMotorConfig(104, "sim-talon-cancoder-test");
    SimTalonFXIO simTalon(
        config,
        frc::sim::DCMotorSim(
            frc::LinearSystemId::DCMotorSystem(
                frc::DCMotor::KrakenX60(1),
                units::kilogram_square_meter_t{0.01},
                1.0),
            frc::DCMotor::KrakenX60(1),
            {0.0, 0.0}));

    simTalon.lastRotations.store(2.0);
    simTalon.lastRPS.store(3.0);

    auto canCoderConfig = MakeCanCoderConfig();
    canCoderConfig.gearing = 100.0;
    canCoderConfig.canCodergearing = 1.0;
    const auto supplier = simTalon.GetSupplierForCancoder(canCoderConfig);
    const auto state = supplier();

    EXPECT_NEAR(state.positionRotations, 200.0, 1e-6);
    EXPECT_NEAR(state.velocityRotations, 300.0, 1e-6);
}

TEST_F(SimSubsystemTest, TalonSetInvertVoltageUpdatesStoredFlag) {
    auto* simTalon = new SimTalonFXIO(
        MakeMotorConfig(103, "sim-talon-invert-test"));

    EXPECT_FALSE(simTalon->invertVoltage);
    simTalon->SetInvertVoltage(true);
    EXPECT_TRUE(simTalon->invertVoltage);
    simTalon->SetInvertVoltage(false);
    EXPECT_FALSE(simTalon->invertVoltage);
}

TEST_F(SimSubsystemTest, TalonHigherReductionLowersMechanismVelocity) {
    const double directVelocity = RunTalonVelocityAfterVoltageStep(1.0, 100_ms);

    RestartTiming();
    PauseTiming();

    const double reducedVelocity = RunTalonVelocityAfterVoltageStep(10.0, 100_ms);

    EXPECT_LT(std::abs(reducedVelocity), std::abs(directVelocity));
}

TEST_F(SimSubsystemTest, ElevatorStartupDeltaDependsOnFpgaUptime) {
    const auto baseline = RunElevatorStep(0_s);

    RestartTiming();
    PauseTiming();

    const auto delayed = RunElevatorStep(2_s);

    ASSERT_EQ(baseline.size(), delayed.size());
    EXPECT_GT(std::abs(delayed[0].rawRotorPosition - baseline[0].rawRotorPosition), 1e-6);
    EXPECT_GT(std::abs(delayed[0].velocityUnitsPerSecond -
                       baseline[0].velocityUnitsPerSecond),
              1e-6);
}

TEST_F(SimSubsystemTest, ElevatorFollowerSignsMatchInversionConfig) {
    const auto inputs = RunElevatorStep(0_s);

    ASSERT_EQ(inputs.size(), 3u);
    EXPECT_NEAR(inputs[1].rawRotorPosition, -inputs[0].rawRotorPosition, 1e-6);
    EXPECT_NEAR(inputs[2].rawRotorPosition, inputs[0].rawRotorPosition, 1e-6);
}

TEST_F(SimSubsystemTest, ElevatorHigherReductionLowersVelocity) {
    const double directVelocity = RunElevatorVelocityAfterVoltageStep(1.0, 100_ms);

    RestartTiming();
    PauseTiming();

    const double reducedVelocity = RunElevatorVelocityAfterVoltageStep(10.0, 100_ms);

    EXPECT_LT(std::abs(reducedVelocity), std::abs(directVelocity));
}

}  // namespace
