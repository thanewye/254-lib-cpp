#pragma once

#include <frc/geometry/Translation2d.h>
#include <units/length.h>

namespace field_constants {
    inline constexpr units::meter_t kFieldLength = 16.5410_m;
    inline constexpr units::meter_t kFieldWidth = 8.0693_m;
    inline constexpr frc::Translation2d kBlueLeftCorner{0.0000_m, 8.0693_m};
    inline constexpr frc::Translation2d kBlueRightCorner{0.0000_m, 0.0000_m};
    inline constexpr frc::Translation2d kRedLeftCorner{16.5410_m, 8.0693_m};
    inline constexpr frc::Translation2d kRedRightCorner{16.5410_m, 0.0000_m};
    inline constexpr frc::Translation2d kRedHubCenter{11.9154_m, 4.0348_m};
    inline constexpr frc::Translation2d kBlueHubCenter{4.6256_m, 4.0348_m};
    inline constexpr frc::Translation2d kFieldCenter{8.2705_m, 4.0348_m};
    inline constexpr frc::Translation2d kBlueTowerEdgeCenter{0.5738_m, 3.7455_m};
    inline constexpr frc::Translation2d kRedTowerEdgeCenter{15.9672_m, 4.3238_m};
    inline constexpr frc::Translation2d kBlueRightTrenchCenter{4.6256_m, 0.6393_m};
    inline constexpr frc::Translation2d kBlueLeftTrenchCenter{4.6256_m, 7.4300_m};
    inline constexpr frc::Translation2d kRedRightTrenchCenter{11.9154_m, 0.6393_m};
    inline constexpr frc::Translation2d kRedLeftTrenchCenter{11.9154_m, 7.4300_m};
    inline constexpr frc::Translation2d kBlueDepotCenter{0.3429_m, 5.9634_m};
    inline constexpr frc::Translation2d kRedDepotCenter{16.1981_m, 2.1062_m};
    inline constexpr frc::Translation2d kBlueLeftBumpCenter{4.6256_m, 5.4747_m};
    inline constexpr frc::Translation2d kBlueRightBumpCenter{4.6256_m, 2.5946_m};
    inline constexpr frc::Translation2d kRedLeftBumpCenter{11.9154_m, 5.4747_m};
    inline constexpr frc::Translation2d kRedRightBumpCenter{11.9154_m, 2.5946_m};
    inline constexpr frc::Translation2d kBlueRightTrenchLeftEdge{4.6256_m, 0.0000_m};
    inline constexpr frc::Translation2d kBlueRightTrenchRightEdge{4.6256_m, 1.2786_m};
    inline constexpr frc::Translation2d kBlueLeftTrenchLeftEdge{4.6256_m, 6.7897_m};
    inline constexpr frc::Translation2d kBlueLeftTrenchRightEdge{4.6256_m, 8.0693_m};
    inline constexpr frc::Translation2d kRedRightTrenchLeftEdge{11.9154_m, 0.0000_m};
    inline constexpr frc::Translation2d kRedRightTrenchRightEdge{11.9154_m, 1.2786_m};
    inline constexpr frc::Translation2d kRedLeftTrenchLeftEdge{11.9154_m, 6.7897_m};
    inline constexpr frc::Translation2d kRedLeftTrenchRightEdge{11.9154_m, 8.0693_m};
    inline constexpr frc::Translation2d kBlueTowerLeftCorner{0.5738_m, 3.2502_m};
    inline constexpr frc::Translation2d kBlueTowerRightCorner{0.5738_m, 4.2408_m};
    inline constexpr frc::Translation2d kRedTowerLeftCorner{15.9672_m, 3.8285_m};
    inline constexpr frc::Translation2d kRedTowerRightCorner{15.9672_m, 4.8191_m};
    inline constexpr units::meter_t kGoalHeight = units::inch_t{60.0};
    inline constexpr units::meter_t kGoalRadius = units::inch_t{20.5};
} // namespace field_constants
