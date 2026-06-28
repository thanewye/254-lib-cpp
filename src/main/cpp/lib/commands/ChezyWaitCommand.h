#pragma once

#include <frc2/command/Command.h>
#include <units/time.h>

class ChezyWaitCommand : public frc2::Command {
public:
    ChezyWaitCommand(units::second_t duration, units::second_t tolerance = 0_s);

    void Initialize() override;
    void End(bool interrupted) override;
    bool IsFinished() override;
    bool RunsWhenDisabled() const override;

private:
    units::second_t m_duration;
    units::second_t m_tolerance;
    units::second_t m_startTime{0_s};
};
