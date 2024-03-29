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
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
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
tap::algorithms::SimplePIDConfig config = {0.0f, 0.0f, 0.0f, 5'000.0f, 10'000.0f, 0.0f, 0.0f};
tap::algorithms::SimplePID pid = tap::algorithms::SimplePID(config);
tap::motor::DjiMotor rf_motor = tap::motor::DjiMotor(drivers, tap::motor::MOTOR1, tap::can::CanBus::CAN_BUS1, true, "Front Right Wheel");
tap::motor::DjiMotor lf_motor = tap::motor::DjiMotor(drivers, tap::motor::MOTOR2, tap::can::CanBus::CAN_BUS1, false, "Front Left Wheel");
float setpoint = 0;

// ROS Stuff
namespace ros
{
	using modmHardware = ModmHardware<modm::platform::Uart7>;
	using ModmNodeHandle = NodeHandle_<modmHardware>;
}
ros::ModmNodeHandle nh;


std_msgs::Int16 encoder_msg;
ros::Publisher pub_encoder("encoder", &encoder_msg);

void motor_constant_callback(const std_msgs::Float32MultiArray &msg)
{
    pid.setP(msg.data[0]);
    pid.setI(msg.data[1]);
    pid.setD(msg.data[2]);
    pid.setMaxICumulative(msg.data[3]);
    pid.setMaxOutput(msg.data[4]);
    pid.setErrDeadzone(msg.data[5]);
    setpoint = msg.data[6];
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


    Board::initialize();
    initializeIo(drivers);
    nh.initNode();
    rf_motor.initialize();
    lf_motor.initialize();

	ros::Subscriber<std_msgs::Float32MultiArray> sub_kp("/constants", &motor_constant_callback);
    nh.subscribe(sub_kp);

    nh.advertise(pub_encoder);

#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    uint16_t i = 0;
    uint32_t last_time = tap::arch::clock::getTimeMilliseconds(), last_spin = last_time;
    while (1)
    {
        PROFILE(drivers->profiler, updateIo, (drivers));
        if (sendMotorTimeout.execute())
        {
            //OFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
            drivers->canRxHandler.pollCanData();
        }

        uint32_t now = tap::arch::clock::getTimeMilliseconds();
        //slow down the ros communication, too fast will mess it up
        if (now - last_spin > 500)
        {
            encoder_msg.data = rf_motor.getShaftRPM();
            //encoder_msg.data = i;
            pub_encoder.publish(&encoder_msg);
            nh.spinOnce();
            last_spin = now;
        }

        //encoder data arrives a 1 kHz, run at 1/4 that
        now = tap::arch::clock::getTimeMilliseconds();
        if (now - last_time > 10)
        {
            rf_motor.setDesiredOutput(pid.runControllerDerivateError(setpoint - (float)rf_motor.getShaftRPM(), now/1000.0f - last_time/1000.0f));
            last_time = now;
        }
        modm::delay_us(10);
        i++;
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
    for (int led = tap::gpio::Leds::LedPin::A; led != tap::gpio::Leds::LedPin::Green; led++)
    {
        drivers->leds.set((tap::gpio::Leds::LedPin)led, true);
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
