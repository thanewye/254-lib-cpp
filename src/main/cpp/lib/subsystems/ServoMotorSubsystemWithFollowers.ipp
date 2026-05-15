#pragma once

#include <cassert>
#include <utility>

#include <frc2/command/Commands.h>

#include "ServoMotorSubsystemWithFollowers.h"
#include "akit/Logger.h"

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
ServoMotorSubsystemWithFollowers<pos_t, T, U>::ServoMotorSubsystemWithFollowers(
    ServoMotorSubsystemWithFollowersConfig<pos_t>& config, T leaderInputs, U* leaderIO,
    std::vector<T> followerInputs, std::vector<U*> followerIOs)
    : ServoMotorSubsystem<pos_t, T, U>(config, std::move(leaderInputs), leaderIO)
      , leaderConfig(config)
      , followerConfigs(leaderConfig.followers)
      , followerInputs(std::move(followerInputs))
      , followerIOs(std::move(followerIOs)) {
    assert(this->followerInputs.size() == this->followerIOs.size());
    assert(this->followerInputs.size() == this->followerConfigs.size());
    followerLogKeys.reserve(this->followerInputs.size());
    for (size_t i = 0; i < this->followerIOs.size(); i++) {
        MotorIO* follower = this->followerIOs[i];
        follower->Follow(leaderConfig.talonCANID, followerConfigs[i].inverted);
        followerLogKeys.push_back(this->LogKey("/follower" + std::to_string(i)));
    }
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystemWithFollowers<pos_t, T, U>::Periodic() {
    ServoMotorSubsystem<pos_t, T, U>::Periodic();
    for (size_t i = 0; i < followerConfigs.size(); i++) {
        followerIOs[i]->ReadInputs(followerInputs[i]);
        akit::Logger::ProcessInputs(followerLogKeys[i], followerInputs[i]);
    }
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystemWithFollowers<pos_t, T, U>::SetCurrentPosition(pos_t position) {
    ServoMotorSubsystem<pos_t, T, U>::SetCurrentPosition(position);
    for (auto* follower : followerIOs) {
        follower->SetCurrentPosition(position.value());
    }
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
void ServoMotorSubsystemWithFollowers<pos_t, T, U>::SetCurrentPositionAsZero() {
    ServoMotorSubsystem<pos_t, T, U>::SetCurrentPositionAsZero();
    for (auto* follower : followerIOs) {
        follower->SetCurrentPositionAsZero();
    }
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
pos_t ServoMotorSubsystemWithFollowers<pos_t, T, U>::GetCurrentPosition() const {
    double averagePosition = this->inputs.unitPosition;
    for (const auto& follower : followerInputs) {
        averagePosition += follower.unitPosition;
    }
    return pos_t{averagePosition / (followerIOs.size() + 1)};
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
typename ServoMotorSubsystemWithFollowers<pos_t, T, U>::vel_t
ServoMotorSubsystemWithFollowers<pos_t, T, U>::GetCurrentVelocity() const {
    double averageVelocity = this->inputs.velocityUnitsPerSecond;
    for (const auto& follower : followerInputs) {
        averageVelocity += follower.velocityUnitsPerSecond;
    }
    return vel_t{averageVelocity / (followerIOs.size() + 1)};
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
double ServoMotorSubsystemWithFollowers<pos_t, T, U>::GetAverageStatorCurrentAmps() const {
    return GetStatorCurrentAmps() / (followerIOs.size() + 1);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
double ServoMotorSubsystemWithFollowers<pos_t, T, U>::GetAverageSupplyCurrentAmps() const {
    return GetSupplyCurrentAmps() / (followerIOs.size() + 1);
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
double ServoMotorSubsystemWithFollowers<pos_t, T, U>::GetStatorCurrentAmps() const {
    double totalCurrent = this->inputs.currentStatorAmps;
    for (const auto& follower : followerInputs) {
        totalCurrent += follower.currentStatorAmps;
    }
    return totalCurrent;
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
double ServoMotorSubsystemWithFollowers<pos_t, T, U>::GetSupplyCurrentAmps() const {
    double totalCurrent = this->inputs.currentSupplyAmps;
    for (const auto& follower : followerInputs) {
        totalCurrent += follower.currentSupplyAmps;
    }
    return totalCurrent;
}

template<typename pos_t, IsMotorInputs T, IsMotorIO U>
frc2::CommandPtr ServoMotorSubsystemWithFollowers<pos_t, T, U>::SystemTestCommand(
    std::string_view testName, double dutyCycle, units::second_t duration) {
    std::vector<frc2::CommandPtr> allTests;

    allTests.push_back(frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this] {
            this->SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue::Coast);
            for (auto* followerIo : followerIOs) {
                followerIo->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
                followerIo->SetOpenLoopDutyCycle(0.0);
            }
        }),
        frc2::cmd::Run([this, dutyCycle, name = std::string{testName}] {
            this->SetOpenLoopDutyCycleImpl(dutyCycle);
            akit::Logger::RecordOutput("SystemTest/" + name + "/Leader/StatorCurrent", this->inputs.currentStatorAmps);
            akit::Logger::RecordOutput("SystemTest/" + name + "/Leader/SupplyCurrent", this->inputs.currentSupplyAmps);
            akit::Logger::RecordOutput("SystemTest/" + name + "/Leader/Velocity", this->inputs.velocityUnitsPerSecond);
        }, {this}).WithTimeout(duration)
    ));

    for (size_t i = 0; i < followerIOs.size(); i++) {
        allTests.push_back(frc2::cmd::Sequence(
            frc2::cmd::RunOnce([this, i] {
                this->SetOpenLoopDutyCycleImpl(0.0);
                for (size_t j = 0; j < followerIOs.size(); j++) {
                    if (j != i) {
                        followerIOs[j]->SetOpenLoopDutyCycle(0.0);
                    }
                }
            }),
            frc2::cmd::Run([this, i, dutyCycle, name = std::string{testName}] {
                followerIOs[i]->SetOpenLoopDutyCycle(dutyCycle);
                akit::Logger::RecordOutput("SystemTest/" + name + "/Follower" + std::to_string(i) + "/StatorCurrent",
                                           followerInputs[i].currentStatorAmps);
                akit::Logger::RecordOutput("SystemTest/" + name + "/Follower" + std::to_string(i) + "/SupplyCurrent",
                                           followerInputs[i].currentSupplyAmps);
                akit::Logger::RecordOutput("SystemTest/" + name + "/Follower" + std::to_string(i) + "/Velocity",
                                           followerInputs[i].velocityUnitsPerSecond);
            }, {this}).WithTimeout(duration)
        ));
    }

    return frc2::cmd::Sequence(std::move(allTests))
            .FinallyDo([this](bool interrupted) {
                this->SetOpenLoopDutyCycleImpl(0.0);
                this->SetNeutralModeImpl(ctre::phoenix6::signals::NeutralModeValue::Brake);
                for (size_t i = 0; i < followerIOs.size(); i++) {
                    followerIOs[i]->SetOpenLoopDutyCycle(0.0);
                    followerIOs[i]->SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
                    followerIOs[i]->Follow(leaderConfig.talonCANID, followerConfigs[i].inverted);
                }
            })
            .WithName("Test " + std::string{testName});
}
