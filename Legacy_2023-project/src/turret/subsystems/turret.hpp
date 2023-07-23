#pragma once

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/clock.hpp"
#include "drivers.hpp"

namespace src::turret
{

class TurretSubsystem : public tap::control::Subsystem
{
    public:
        TurretSubsystem(tap::Drivers* drivers) :
            tap::control::Subsystem(drivers),
            pitchMotor(drivers, PITCH_MOTOR_ID, MOTOR_CAN_BUS, true, "Pitch Motor"),
            yawMotor(drivers, YAW_MOTOR_ID, MOTOR_CAN_BUS, true, "Yaw Motor"),
            pitchPID(pitchPIDConfig),
            yawPID(yawPIDConfig),
            pitchPosition(0), yawPosition(0), pitchDesired(0), yawDesired(0), lastPIDUpdate(0),
            enabled(false), calibrated(false)
        {
            //PID controller outputs float, could it be an issue?
            pitchPID.setMaxOutput(2 * GM6020_MAX_OUTPUT);
            yawPID.setMaxOutput(2 * GM6020_MAX_OUTPUT);

            //TODO: set integral runoff limits when tuning
            //pitchPID.setMaxICumulative();
            //yawPID.setMaxICumulative();
        }

        //units of encoder ticks, 8192 max
        static constexpr uint16_t MIN_ANGLE_PITCH_ENC = 0;              //0 degrees
        static constexpr uint16_t MAX_ANGLE_PITCH_ENC = 8192;           //360 degrees
        static constexpr uint16_t MIN_ANGLE_YAW_ENC = 1365;             //60 degrees
        static constexpr uint16_t MAX_ANGLE_YAW_ENC = 3640;             //160 degrees

        //TODO: put this in a motor constants file
        static constexpr uint16_t GM6020_MAX_OUTPUT = 30000;

        void initialize() override;

        void refresh() override;

        //custom functions
        void setDesiredOutput(float pitch, float yaw);
        inline float getPitch() { return 360.0f * pitchPosition / tap::motor::DjiMotor::ENC_RESOLUTION; }
        inline float getYaw() { return 360.0f * yawPosition / tap::motor::DjiMotor::ENC_RESOLUTION; }
        void getSetpoint(float &pitch, float &yaw);

    private:
        //hardware constants
        static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR1;
        static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR2;
        static constexpr tap::can::CanBus MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;

        //software objects
        tap::motor::DjiMotor pitchMotor;
        tap::motor::DjiMotor yawMotor;
        tap::algorithms::SmoothPidConfig pitchPIDConfig = { 1.0f, 0.0f, 0.0f };
        tap::algorithms::SmoothPidConfig yawPIDConfig = { 1.0f, 0.0f, 0.0f };
        tap::algorithms::SmoothPid pitchPID;
        tap::algorithms::SmoothPid yawPID;

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