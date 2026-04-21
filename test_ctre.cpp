#include "ctre/phoenix6/TalonFX.hpp"
void t() {
    ctre::phoenix6::hardware::TalonFX talon{0};
    auto sig = talon.GetPosition();
}
