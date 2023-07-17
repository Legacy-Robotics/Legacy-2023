#include "turret.hpp"

namespace src::turret
{

void TurretSubsystem::initialize()
{
    pitchMotor.initialize();
    yawMotor.initialize();
    idxMotor.initialize();
    rightFireMotor.initialize();
    leftFireMotor.initialize();
}

void TurretSubsystem::refresh()
{

}

void TurretSubsystem::setDesiredAngle(float pitch, float yaw)
{
    pitchMotor.setDesiredOutput(pitch);
    pitchMotor.setDesiredOutput(yaw);
}

}