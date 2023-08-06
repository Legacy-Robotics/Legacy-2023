#include "turret.hpp"

namespace src::turret
{

void TurretSubsystem::initialize()
{
    lastPIDUpdate = tap::arch::clock::getTimeMilliseconds();
    pitchMotor.initialize();
    yawMotor.initialize();
}

void TurretSubsystem::refresh()
{
    //TODO: motor should be calibrated and on angle before running PID
    enabled = drivers->refSerial.controlIsDisabled();
    if (enabled && calibrated)
    {
        uint32_t timeNow = tap::arch::clock::getTimeMilliseconds(), dt = timeNow - lastPIDUpdate;

        //get the actual encoder value, and give the PID controllers the error
        pitchPosition = pitchMotor.getEncoderWrapped();
        yawPosition = pitchMotor.getEncoderWrapped();
        pitchPID.runControllerDerivateError(pitchDesired - pitchPosition, dt);
        pitchPID.runControllerDerivateError(yawDesired - yawPosition, dt);

        //take the output calculated by the PID controller and command it to the motors
        //subtract GM6020_MAX_OUTPUT to create "negative" output from PID
        pitchMotor.setDesiredOutput(pitchPID.getOutput() - GM6020_MAX_OUTPUT);
        yawMotor.setDesiredOutput(yawPID.getOutput() - GM6020_MAX_OUTPUT);
    }
}

/**
 * @brief Sets desired pitch and yaw angles.
 * 
 * Pitch and yaw setpoints will be updated to PID controller in refresh() method.
 * @param pitch Desired pitch angle [0, 360] (degrees)
 * @param yaw Desired yaw angle [0, 360] (degrees)
 * @note pitch 0 degrees = left back, 360 degrees = back right
 * @note yaw 0 degrees = down forward, 360 degrees = down backwards
 * @bug doesn't account for negative rollover, e.g. 1 deg -> 350 deg will go 349 deg instead of 11 
 */
void TurretSubsystem::setDesiredOutput(float pitch, float yaw)
{
    this->pitchDesired = pitch;
    this->yawDesired = yaw;
    pitchMotor.setDesiredOutput(pitch);
    yawMotor.setDesiredOutput(yaw);
}

}