#pragma once

#include <vector>

#include <frc/system/plant/DCMotor.h>
#include <units/moment_of_inertia.h>

#include "lib/subsystems/TalonFXIO.h"

class SimElevator {
 public:
  struct SimElevatorConfig {
    frc::DCMotor motor = frc::DCMotor::KrakenX60();
    double gearing = 1.0;
    units::kilogram_square_meter_t moi{0.001};
    double drumRadiusMeters = 0.02;
    double carriageMassKg = 1.0;
  };

  explicit SimElevator(const SimElevatorConfig& config);

  TalonFXIO* GetLeadIO();
  std::vector<TalonFXIO*> GetFollowerIOs();

  void UpdateSimState();
};
