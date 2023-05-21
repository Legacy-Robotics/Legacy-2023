#include "Turret.hpp"

namespace src::Turret
{

void TurretSubsystem::initialize()
{
    yawMotor.initialize();
    pitchMotor.initialize();
}

void TurretSubsystem::refresh()
{

}

void TurretSubsystem::setDesiredOutput(float yawPower, float pitchPower)
{
    yawMotor.setDesiredOutput(yawPower * MAX_CURRENT_OUTPUT);
    pitchMotor.setDesiredOutput(pitchPower * MAX_CURRENT_OUTPUT);
}

}