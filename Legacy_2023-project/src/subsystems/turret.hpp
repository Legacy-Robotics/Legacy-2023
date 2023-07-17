#pragma once

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"

namespace src::turret
{

class TurretSubsystem : public tap::control::Subsystem
{
    public:
        static constexpr float MAX_CURRENT_OUTPUT = 10000.0f;
        TurretSubsystem(tap::Drivers* drivers) :
            tap::control::Subsystem(drivers),
            pitchMotor(drivers, PITCH_MOTOR_ID, MOTOR_CAN_BUS, true, "Pitch Motor"),
            yawMotor(drivers, YAW_MOTOR_ID, MOTOR_CAN_BUS, false, "Yaw Motor"),
            idxMotor(drivers, IDX_MOTOR_ID, MOTOR_CAN_BUS, false, "Indexer Motor"),
            leftFireMotor(drivers, LEFT_FIRE_MOTOR_ID, MOTOR_CAN_BUS, false, "Left Firing Motor"),
            rightFireMotor(drivers, RIGHT_FIRE_MOTOR_ID, MOTOR_CAN_BUS, false, "Right Firing Motor")
        {}

        void initialize() override;

        void refresh() override;

        //custom functions
        void setDesiredAngle(float pitch, float yaw);
        void setPitch(float pitch);
        void setYaw(float yaw);
        void setFireSpinning(bool spinning);
        void indexRound();

    private:
        //hardware constants
        static constexpr tap::motor::MotorId PITCH_MOTOR_ID = tap::motor::MOTOR1;
        static constexpr tap::motor::MotorId YAW_MOTOR_ID = tap::motor::MOTOR2;
        static constexpr tap::motor::MotorId IDX_MOTOR_ID = tap::motor::MOTOR3;
        static constexpr tap::motor::MotorId LEFT_FIRE_MOTOR_ID = tap::motor::MOTOR4;
        static constexpr tap::motor::MotorId RIGHT_FIRE_MOTOR_ID = tap::motor::MOTOR5;
        static constexpr tap::can::CanBus MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS2;

        //software objects
        tap::motor::DjiMotor pitchMotor;
        tap::motor::DjiMotor yawMotor;
        tap::motor::DjiMotor idxMotor;
        tap::motor::DjiMotor leftFireMotor;
        tap::motor::DjiMotor rightFireMotor;
};

}