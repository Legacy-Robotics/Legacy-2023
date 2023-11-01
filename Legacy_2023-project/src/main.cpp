/*
 * Copyright (c) 2020-2021 Legacy_Robotics
 *
 * This file is part of Legacy_2023.
 *
 * Legacy_2023 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Legacy_2023 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Legacy_2023.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifdef PLATFORM_HOSTED


/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::DISABLED
#define USE_USBCON

#include "tap/board/board.hpp"
#include "robot_control.hpp"
#include "modm/architecture/interface/delay.hpp"

#include "tap/algorithms/smooth_pid.hpp"

/* arch includes ------------------------------------------------------------*/
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/profiler.hpp"

/* communication includes ---------------------------------------------------*/
#include "drivers.hpp"
#include "drivers_singleton.hpp"

/* error handling includes --------------------------------------------------*/
#include "tap/errors/create_errors.hpp"

/* control includes ---------------------------------------------------------*/
#include "tap/architecture/clock.hpp"

/* ros includes ---------------------------------------------------------*/
#include <modm/communication/ros.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.hpp>
#include <ros/node_handle.h>
#include "tap/communication/serial/uart.hpp"
#include "tap/motor/dji_motor.hpp"
#include "tap/communication/serial/uart.hpp"

#include "tap/algorithms/simple_pid.hpp"

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);

src::Drivers *drivers = src::DoNotUse_getDrivers();

//state variables
SimplePIDConfig config = {0.0f, 0.0f, 0.0f, 5'000.0f, 10'000.0f, 0.0f, 0.0f};
SimplePID pid = SimplePID(config);
int32_t setpoint = 0;

// ROS Stuff
namespace ros
{
	using modmHardware = ModmHardware<modm::platform::Uart7>;
	using ModmNodeHandle = NodeHandle_<modmHardware>;
}
ros::ModmNodeHandle nh;


std_msgs::Int32 encoder_msg;
ros::Publisher pub_encoder("encoder", &encoder_msg);

void motor_state(const std_msgs::UInt& msg) {
    drivers->leds.set(tap::gpio::Leds::B, msg.data);
}

void motor_kp_callback(const std_msgs::Float32 &msg)
{
    pid.setP(msg.data);
}

void motor_ki_callback(const std_msgs::Float32 &msg)
{
    pid.setI(msg.data);
}

void motor_kd_callback(const std_msgs::Float32 &msg)
{
    pid.setD(msg.data);
}

void motor_maxi_callback(const std_msgs::Float32 &msg)
{
    pid.setMaxICumulative(msg.data);
}

void motor_max_out_callback(const std_msgs::Float32 &msg)
{
    pid.setMaxOutput(msg.data);
}

void motor_dead_callback(const std_msgs::Float32 &msg)
{
    pid.setErrDeadzone(msg.data);
}

void motor_setpoint_callback(const std_msgs::Float32 &msg)
{
    setpoint = msg; 
}

int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif /*PLATFORM HOSTED*/

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    src::Control::initializeSubsystemCommands(drivers);
    tap::motor::DjiMotor rf_motor = tap::motor::DjiMotor(drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS1, true, "Front Right Wheel");

    Board::initialize();
    initializeIo(drivers);
    nh.initNode();

	ros::Subscriber<std_msgs::Float32> sub_kp("/constants/kp", &motor_kp_callback);
	ros::Subscriber<std_msgs::Float32> sub_ki("/constants/ki", &motor_ki_callback);
	ros::Subscriber<std_msgs::Float32> sub_kd("/constants/kd", &motor_kd_callback);
	ros::Subscriber<std_msgs::Float32> sub_maxi("/constants/maxi", &motor_maxi_callback);
	ros::Subscriber<std_msgs::Float32> sub_maxo("/constants/maxo", &motor_max_out_callback);
	ros::Subscriber<std_msgs::Float32> sub_dead("/constants/dead", &motor_dead_callback);
	ros::Subscriber<std_msgs::Float32> sub_setp("/setpoint", &motor_setpoint_callback);

    nh.subscribe(sub_kp);
    nh.subscribe(sub_ki);
    nh.subscribe(sub_kd);
    nh.subscribe(sub_maxi);
    nh.subscribe(sub_maxo);
    nh.subscribe(sub_dead);
    nh.subscribe(sub_setp);

    nh.advertise(pub_encoder);

#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    uint16_t i = 0;
    while (1)
    {
        PROFILE(drivers->profiler, updateIo, (drivers));
        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            //PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            //PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
        }
        drivers->canRxHandler.pollCanData();

        //slow down the ros communication, too fast will mess it up
        if (i % 5000 == 0)
        {
            encoder_msg.data = rf_motor.getShaftRPM();
            pub_encoder.publish(&encoder_msg);
            nh.spinOnce();
        }

        //encoder data arrives a 1 kHz, run at 1/4 that
        if (i % 4000 == 0)
        {
            rf_motor.setDesiredOutput(pid.runControllerDerivateError(setpoint - rf_motor.getShaftRPM()));
        }
        modm::delay_us(10);
    }
    return 0;
}

static void initializeIo(src::Drivers *drivers)
{
    drivers->analog.init();
    drivers->pwm.init();
    drivers->digital.init();
    drivers->leds.init();
    drivers->can.initialize();
    drivers->errorController.init();
    drivers->remote.initialize();
    drivers->mpu6500.init(0,0,0);
    drivers->refSerial.initialize();
    drivers->vtm.initialize();
    drivers->terminalSerial.initialize();
    drivers->schedulerTerminalHandler.init();
    drivers->djiMotorTerminalSerialHandler.init();
    //ROS UART port initialization
    drivers->uart.init<tap::communication::serial::Uart::UartPort::Uart7, 115200>();
    //turn off all LEDs
    for (int led = tap::gpio::Leds::A; led != tap::gpio::Leds::Green; led++)
    {
        drivers->leds.set(led, true);
    }
}

static void updateIo(src::Drivers *drivers)
{
#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::updateSims();
#endif

    drivers->canRxHandler.pollCanData();
    drivers->refSerial.updateSerial();
    drivers->vtm.updateSerial();
    drivers->mpu6500.read();
}
