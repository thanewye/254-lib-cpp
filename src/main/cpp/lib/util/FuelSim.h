#pragma once

#include <functional>
#include <vector>

#include <frc/geometry/Pose2d.h>

class FuelSim {
 public:
  struct Fuel {
    frc::Pose2d pose;
    bool available = true;
  };

  static FuelSim& GetInstance();

  void Reset();
  void Update();
  void RegisterRobot(int id, std::function<frc::Pose2d()> poseSupplier);
  const std::vector<Fuel>& GetFuels() const;

 private:
  FuelSim() = default;
  std::vector<Fuel> m_fuels;
};
