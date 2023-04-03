#include "DriveTrain.hpp"

namespace src::DriveTrain
{

    void DriveTrainSubsystem::initialize()
    {
        rightFrontMotor.initialize();
        leftFrontMotor.initialize();
        leftBackMotor.initialize();
        rightBackMotor.initialize();
    }

    void DriveTrainSubsystem::refresh()
    {

    }

    void  DriveTrainSubsystem::setDesiredOutput(float x, float y, float rot)
    {
        
        
        rightFrontMotor.setDesiredOutput((y-x-rot) * MAX_CURRENT_OUTPUT);
        leftFrontMotor.setDesiredOutput((y+x+rot) * MAX_CURRENT_OUTPUT);
        leftBackMotor.setDesiredOutput((y-x+rot) * MAX_CURRENT_OUTPUT);
        rightBackMotor.setDesiredOutput((y+x-rot) * MAX_CURRENT_OUTPUT);
    }
}