#include "akit/telemetry/LoggedDriverStation.h"

#include <string>
#include <vector>

#include <frc/DriverStation.h>
#include <frc/GenericHID.h>
#include <frc/Joystick.h>
#include <frc/simulation/DriverStationSim.h>
#include <hal/DriverStation.h>

namespace akit {
    void LoggedDriverStation::SaveToLog(const LogTable& table) {
        int32_t status = 0;

        table.Put("AllianceStation", HAL_GetAllianceStation(&status));
        table.Put("EventName", std::string_view{frc::DriverStation::GetEventName()});
        table.Put("GameSpecificMessage", std::string_view{frc::DriverStation::GetGameSpecificMessage()});
        table.Put("MatchNumber", frc::DriverStation::GetMatchNumber());
        table.Put("ReplayNumber", frc::DriverStation::GetReplayNumber());
        table.Put("MatchType", frc::DriverStation::GetMatchType());
        table.Put("MatchTime", frc::DriverStation::GetMatchTime());

        table.Put("Enabled", frc::DriverStation::IsEnabled());
        table.Put("Autonomous", frc::DriverStation::IsAutonomous());
        table.Put("Test", frc::DriverStation::IsTest());
        table.Put("EmergencyStop", frc::DriverStation::IsEStopped());
        table.Put("FMSAttached", frc::DriverStation::IsFMSAttached());
        table.Put("DSAttached", frc::DriverStation::IsDSAttached());

        for (int id = 0; id < frc::DriverStation::kJoystickPorts; id++) {
            LogTable joystickTable = table.GetSubtable("Joystick" + std::to_string(id));

            HAL_JoystickDescriptor desc{};
            HAL_GetJoystickDescriptor(id, &desc);
            joystickTable.Put("Name", std::string_view{desc.name});
            joystickTable.Put("Type", static_cast<frc::GenericHID::HIDType>(desc.type));
            joystickTable.Put("Xbox", static_cast<bool>(desc.isXbox));
            joystickTable.Put("ButtonCount", desc.buttonCount);

            HAL_JoystickButtons buttons{};
            HAL_GetJoystickButtons(id, &buttons);
            joystickTable.Put("ButtonValues", static_cast<int64_t>(buttons.buttons));

            HAL_JoystickPOVs povs{};
            HAL_GetJoystickPOVs(id, &povs);
            std::vector<int> povValues(povs.povs, povs.povs + povs.count);
            joystickTable.Put("POVs", std::span<const int>(povValues));

            HAL_JoystickAxes axes{};
            HAL_GetJoystickAxes(id, &axes);
            std::vector<float> axisValues(axes.axes, axes.axes + axes.count);
            joystickTable.Put("AxisValues", std::span<const float>(axisValues));

            std::vector<frc::Joystick::AxisType> axisTypes(axes.count);
            for (int i = 0; i < axes.count; i++) {
                axisTypes[i] = static_cast<frc::Joystick::AxisType>(desc.axisTypes[i]);
            }
            joystickTable.Put("AxisTypes", std::span<const frc::Joystick::AxisType>(axisTypes));
        }
    }

    void LoggedDriverStation::ReplayFromLog(const LogTable& table) {
        using frc::sim::DriverStationSim;

        const auto allianceStation = table.Get("AllianceStation", HAL_AllianceStationID_kUnknown);
        DriverStationSim::SetAllianceStationId(allianceStation);
        DriverStationSim::SetEventName(table.Get("EventName", std::string{}));
        DriverStationSim::SetGameSpecificMessage(table.Get("GameSpecificMessage", std::string{}));
        DriverStationSim::SetMatchNumber(table.Get("MatchNumber", 0));
        DriverStationSim::SetReplayNumber(table.Get("ReplayNumber", 0));
        const auto matchType = table.Get("MatchType", frc::DriverStation::MatchType::kNone);
        DriverStationSim::SetMatchType(matchType);
        DriverStationSim::SetMatchTime(table.Get("MatchTime", -1.0));

        const bool dsAttached = table.Get("DSAttached", false);
        DriverStationSim::SetEnabled(table.Get("Enabled", false));
        DriverStationSim::SetAutonomous(table.Get("Autonomous", false));
        DriverStationSim::SetTest(table.Get("Test", false));
        DriverStationSim::SetEStop(table.Get("EmergencyStop", false));
        DriverStationSim::SetFmsAttached(table.Get("FMSAttached", false));
        DriverStationSim::SetDsAttached(dsAttached);

        for (int id = 0; id < frc::DriverStation::kJoystickPorts; id++) {
            const LogTable joystickTable = table.GetSubtable("Joystick" + std::to_string(id));
            DriverStationSim::SetJoystickName(id, joystickTable.Get("Name", std::string{}));

            const auto joystickType = joystickTable.Get("Type", frc::GenericHID::HIDType::kUnknown);
            DriverStationSim::SetJoystickType(id, joystickType);
            DriverStationSim::SetJoystickIsXbox(id, joystickTable.Get("Xbox", false));
            DriverStationSim::SetJoystickButtonCount(id, joystickTable.Get("ButtonCount", 0));
            DriverStationSim::SetJoystickButtons(
                id, static_cast<uint32_t>(joystickTable.Get("ButtonValues", static_cast<int64_t>(0))));

            const auto povValues = joystickTable.Get("POVs", std::span<const int>{});
            const auto povCount = static_cast<int>(povValues.size());
            DriverStationSim::SetJoystickPOVCount(id, povCount);
            for (int pov = 0; pov < povCount; pov++) {
                DriverStationSim::SetJoystickPOV(id, pov, povValues[pov]);
            }

            const auto axisValues = joystickTable.Get("AxisValues", std::vector<float>{});
            auto axisTypes = joystickTable.Get("AxisTypes", std::span<const frc::Joystick::AxisType>{});
            axisTypes.resize(axisValues.size(), frc::Joystick::AxisType::kXAxis);

            const auto axisCount = static_cast<int>(axisValues.size());
            DriverStationSim::SetJoystickAxisCount(id, axisCount);
            for (int axis = 0; axis < axisCount; axis++) {
                DriverStationSim::SetJoystickAxis(id, axis, axisValues[axis]);
                DriverStationSim::SetJoystickAxisType(id, axis, axisTypes[axis]);
            }
        }

        if (dsAttached) {
            DriverStationSim::NotifyNewData();
        }
    }
}
