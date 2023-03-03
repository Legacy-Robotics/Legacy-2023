#include "Turret.hpp"

namespace src::Turret{
    void TurretSubsystem::initialize(){
        pitchMotor.initialize();
        yawMotor.initialize();
        shooterLeftMotor.initialize();
        shooterRightMotor.initialize();
    }

    void TurretSubsystem::refresh()
    {

    }
}