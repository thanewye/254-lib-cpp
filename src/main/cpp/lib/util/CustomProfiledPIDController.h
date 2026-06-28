#pragma once

#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/math.h>

template<class Distance> class CustomProfiledPIDController {
    using Distance_t = units::unit_t<Distance>;
    using Velocity_t = units::unit_t<units::compound_unit<Distance, units::inverse<units::seconds>>>;
    using Constraints = frc::TrapezoidProfile<Distance>::Constraints;
    using State = frc::TrapezoidProfile<Distance>::State;

public:
    CustomProfiledPIDController(double kP, double kI, double kD, Constraints constraints, double period = 0.02)
        : m_controller(kP, kI, kD, units::second_t{period})
        , m_constraints(constraints)
        , m_profile(constraints) {}

    CustomProfiledPIDController(double kP, double kI, double kD, double kV, Constraints constraints, double period = 0.02)
        : CustomProfiledPIDController(kP, kI, kD, constraints, period) {
        m_kV = kV;
    }

    void SetVelocityFF(double kV) { m_kV = kV; }

    void SetConstraints(Constraints constraints) {
        m_constraints = constraints;
        m_profile = frc::TrapezoidProfile<Distance>(constraints);
    }

    Constraints GetConstraints() const { return m_constraints; }

    void SetGoal(Distance_t position, Velocity_t velocity = Velocity_t{0}) { m_goal = State{position, velocity}; }

    void SetGoal(State goal) { m_goal = goal; }

    State GetGoal() const { return m_goal; }
    State GetSetpoint() const { return m_setpoint; }

    void Reset(Distance_t position, Velocity_t velocity = Velocity_t{0}) {
        m_controller.Reset();
        m_setpoint = State{position, velocity};
    }

    double Calculate(Distance_t measurement) {
        m_setpoint = m_profile.Calculate(m_controller.GetPeriod(), m_setpoint, m_goal);
        double positionOutput = m_controller.Calculate(measurement.value(), m_setpoint.position.value());
        double velocityFF = m_kV * m_setpoint.velocity.value();
        return positionOutput + velocityFF;
    }

    double Calculate(Distance_t measurement, State goal) {
        SetGoal(goal);
        return Calculate(measurement);
    }

    bool AtGoal(double positionTolerance) const { return units::math::abs(m_goal.position - m_setpoint.position).value() < positionTolerance; }

    frc::PIDController& GetPIDController() { return m_controller; }

private:
    frc::PIDController m_controller;
    Constraints m_constraints;
    frc::TrapezoidProfile<Distance> m_profile;

    State m_goal{};
    State m_setpoint{};

    double m_kV = 0.0;
};
