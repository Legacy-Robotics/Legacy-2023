#pragma once

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/communication/serial/remote.hpp"

namespace src::Turret
{

class TurretSubsystem : public tap::control::Subsystem
{
    public:

        static constexpr float MAX_CURRENT_OUTPUT = 10000.0f;
        TurretSubsystem(tap::Drivers* drivers) :
            tap::control::Subsystem(drivers),
            yawMotor(drivers, YAW_MOTOR_ID, MOTOR_CAN_BUS, false, "Yaw Motor"),
            pitchMotor(drivers, PITCH_MOTOR_ID, MOTOR_CAN_BUS, false, "Pitch Motor")
        {}

        void initialize() override;

        void refresh() override;

        //custom functions
        void setDesiredOutput(float yawPower, float pitchPower);

    private:
        //hardware constants
        static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR5;
        static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR6;
        static constexpr tap::can::CanBus MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;

        //software objects
        tap::motor::DjiMotor yawMotor;
        tap::motor::DjiMotor pitchMotor;
};

}