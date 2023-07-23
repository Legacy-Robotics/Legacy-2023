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
        pitchMotor.setDesiredOutput(pitchPID.getOutput());
        yawMotor.setDesiredOutput(yawPID.getOutput());
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
    //convert float degrees to angle here because called less frequently than refresh
    uint16_t pitchTemp = (pitch / 360) * tap::motor::DjiMotor::ENC_RESOLUTION;
    uint16_t yawTemp = (yaw / 360) * tap::motor::DjiMotor::ENC_RESOLUTION;

    //clamp values to angle limits
    pitchDesired = std::clamp<uint16_t>(pitchTemp, MIN_ANGLE_PITCH_ENC, MAX_ANGLE_PITCH_ENC);
    yawDesired = std::clamp<uint16_t>(yawTemp, MIN_ANGLE_YAW_ENC, MAX_ANGLE_YAW_ENC);
}

}