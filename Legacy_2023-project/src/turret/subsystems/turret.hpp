#pragma once

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithims/smooth_pid.hpp"
#include "tap/architecture/clock.hpp"
#include "drivers.hpp"

namespace src::turret
{

class TurretSubsystem : public tap::control::Subsystem
{
    public:
        static constexpr float MAX_CURRENT_OUTPUT = 10000.0f;
        static constexpr uint16_t MIN_ANGLE_PITCH_ENC = 0;              //0 degrees
        static constexpr uint16_t MAX_ANGLE_PITCH_ENC = 8192;           //360 degrees
        static constexpr uint16_t MIN_ANGLE_YAW_ENC = 1365;             //60 degrees
        static constexpr uint16_t MAX_ANGLE_YAW_ENC = 3640;             //160 degrees
        TurretSubsystem(tap::Drivers* drivers) :
            tap::control::Subsystem(drivers),
            calibrated(false),
            enabled(false),
            pitchMotor(drivers, PITCH_MOTOR_ID, MOTOR_CAN_BUS, true, "Pitch Motor"),
            yawMotor(drivers, YAW_MOTOR_ID, MOTOR_CAN_BUS, false, "Yaw Motor"),
        {}

        void initialize() override;

        void refresh() override;

        //custom functions
        void setDesiredOutput(float pitch, float yaw);

    private:
        //hardware constants
        static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR1;
        static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR2;

        //software objects
        tap::motor::DjiMotor pitchMotor;
        tap::motor::DjiMotor yawMotor;
        tap::algorithims::SmoothPid pitchPID;
        tap::algorithims::SmoothPidConfig pitchPIDConfig;
        tap::algorithims::SmoothPid yawPID;
        tap::algorithims::SmoothPidConfig yawPIDConfig;

        //member variables
        uint16_t pitchPosition;
        uint16_t yawPosition;
        uint16_t pitchDesired;
        uint16_t yawDesired;
        uint32_t lastPIDUpdate;
        bool enabled;
        bool calibrated;
};

}