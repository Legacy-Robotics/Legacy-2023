#pragma once

#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/communication/serial/remote.hpp"

namespace src::Drivetrain
{

class DrivetrainSubsystem : public tap::control::Subsystem
{
    public:

        static constexpr float MAX_CURRENT_OUTPUT = 10000.0f;
        DrivetrainSubsystem(tap::Drivers* drivers) :
            tap::control::Subsystem(drivers),
            rightFrontMotor(drivers, RIGHT_FRONT_MOTOR_ID, MOTOR_CAN_BUS, true, "Front Right Wheel"),
            leftFrontMotor(drivers, LEFT_FRONT_MOTOR_ID, MOTOR_CAN_BUS, false, "Front Left Wheel"),
            leftBackMotor(drivers, LEFT_BACK_MOTOR_ID, MOTOR_CAN_BUS, false, "Back Left Wheel"),
            rightBackMotor(drivers, RIGHT_BACK_MOTOR_ID, MOTOR_CAN_BUS, true, "Back Right Wheel")
        {}

        void initialize() override;

        void refresh() override;

        //custom functions
        void setDesiredOutput(float x, float y, float z);

    private:
        //hardware constants
        static constexpr tap::motor::MotorId RIGHT_FRONT_MOTOR_ID = tap::motor::MOTOR1;
        static constexpr tap::motor::MotorId LEFT_FRONT_MOTOR_ID = tap::motor::MOTOR2;
        static constexpr tap::motor::MotorId LEFT_BACK_MOTOR_ID = tap::motor::MOTOR3;
        static constexpr tap::motor::MotorId RIGHT_BACK_MOTOR_ID = tap::motor::MOTOR4;
        static constexpr tap::can::CanBus MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;

        //software objects
        tap::motor::DjiMotor rightFrontMotor;
        tap::motor::DjiMotor leftFrontMotor;
        tap::motor::DjiMotor leftBackMotor;
        tap::motor::DjiMotor rightBackMotor;
};

}