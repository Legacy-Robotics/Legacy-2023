#include "chassis.hpp"

namespace src::chassis
{

void ChassisSubsystem::initialize()
{
    rightFrontMotor.initialize();
    leftFrontMotor.initialize();
    leftBackMotor.initialize();
    rightBackMotor.initialize();
}

void ChassisSubsystem::refresh()
{

}

void ChassisSubsystem::setDesiredOutput(float x, float y, float rot)
{
    rightFrontMotor.setDesiredOutput((y-x-rot) * MAX_CURRENT_OUTPUT);
    leftFrontMotor.setDesiredOutput((y+x+rot) * MAX_CURRENT_OUTPUT);
    leftBackMotor.setDesiredOutput((y-x+rot) * MAX_CURRENT_OUTPUT);
    rightBackMotor.setDesiredOutput((y+x-rot) * MAX_CURRENT_OUTPUT);
}

}