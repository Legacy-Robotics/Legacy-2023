#pragma
#include "tap/control/subsystem.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/communication/serial/remote.hpp"


namespace src::Turret{
    class TurretSubsystem : public tap::control::Subsystem{
        private:
            static constexpr tap::motor::MotorId PITCH_ID = tap::motor::MOTOR5;
            static constexpr tap::motor::MotorId YAW_ID = tap::motor::MOTOR5;
            static constexpr tap::motor::MotorId SHOOT_LEFT_ID = tap::motor::MOTOR7;
            static constexpr tap::motor::MotorId SHOOT_RIGHT_ID = tap::motor::MOTOR8;
            static constexpr tap::can::CanBus MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;

             //software objects
            tap::motor::DjiMotor pitchMotor;
            tap::motor::DjiMotor yawMotor;
            tap::motor::DjiMotor shooterLeftMotor;
            tap::motor::DjiMotor shooterRightMotor;
        

        public:
            static constexpr float MAX_CURRENT_OUTPUT = 10000.0f;
            TurretSubsystem(tap::Drivers *drivers) :
                tap::control::Subsystem(drivers),
                pitchMotor(drivers, PITCH_ID, MOTOR_CAN_BUS, false, "Pitch Motor"),
                yawMotor(drivers, YAW_ID, MOTOR_CAN_BUS, false, "Yaw Motor"),
                shooterLeftMotor(drivers, SHOOT_LEFT_ID, MOTOR_CAN_BUS, false, "Shooter Left Motor"),
                shooterRightMotor(drivers, SHOOT_RIGHT_ID, MOTOR_CAN_BUS, false, "Yaw Motor")
            {}


        
            void initialize() override;

            void refresh() override;

            //custom functions

    };
}