#include "drivetrain.hpp"

namespace src::Drivetrain
{

void DrivetrainSubsystem::initialize()
{
    rightFrontMotor.initialize();
    leftFrontMotor.initialize();
    leftBackMotor.initialize();
    rightBackMotor.initialize();
}

void DrivetrainSubsystem::refresh()
{

}

void DrivetrainSubsystem::setDesiredOutput(float x, float y, float rot)
{
    /*int16_t y = left_vert;
    int16_t x = left_horiz;
    int16_t rot = right_horiz;*/
    /*if(remote.keyPressed(tap::Remote::Key::W))
    {
        y = 1.0;
    }
    if(remote.keyPressed(tap::Remote::Key::S))
    {
        y = -1.0;
    }
    if(remote.keyPressed(tap::Remote::Key::A))
    {
        x = -1.0;
    }
    if(remote.keyPressed(tap::Remote::Key::D))
    {
        x = 1.0;
    }
    if(remote.keyPressed(tap::Remote::Key::Q))
    {
        rot = -1.0;
    }
    if(remote.keyPressed(tap::Remote::Key::E))
    {

    }*/
    rightFrontMotor.setDesiredOutput((y-x-rot) * MAX_CURRENT_OUTPUT);
    leftFrontMotor.setDesiredOutput((y+x+rot) * MAX_CURRENT_OUTPUT);
    leftBackMotor.setDesiredOutput((y-x+rot) * MAX_CURRENT_OUTPUT);
    rightBackMotor.setDesiredOutput((y+x-rot) * MAX_CURRENT_OUTPUT);
}

}