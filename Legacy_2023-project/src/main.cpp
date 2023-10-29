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

#undef	MODM_LOG_LEVEL
#define	MODM_LOG_LEVEL modm::log::DISABLED

/* hosted environment (simulator) includes --------------------------------- */
#include <iostream>

#include "tap/communication/tcp-server/tcp_server.hpp"
#include "tap/motor/motorsim/sim_handler.hpp"
#endif

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
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <ros/node_handle.h>
#include "tap/communication/serial/uart.hpp"
#include "tap/motor/dji_motor.hpp"

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);
// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);


// ROS Stuff
namespace ros
{
	using modmHardware = ModmHardware<modm::platform::Uart7>;
	using ModmNodeHandle = NodeHandle_<modmHardware>;
}
ros::ModmNodeHandle nh;


std_msgs::UInt16 encoder_msg;
ros::Publisher pub_encoder("/encoder", &encoder_msg);

src::Drivers *drivers = src::DoNotUse_getDrivers();
void message_cb(const std_msgs::Bool& msg) {
    drivers->leds.set(tap::gpio::Leds::A, !(msg.data));
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

	ros::Subscriber<std_msgs::Bool> sub_led("/led/one",   &message_cb);
    nh.subscribe(sub_led);
    nh.advertise(pub_encoder);

    drivers->leds.set(tap::gpio::Leds::A, true);
    drivers->leds.set(tap::gpio::Leds::B, true);
    drivers->leds.set(tap::gpio::Leds::C, true);
    drivers->leds.set(tap::gpio::Leds::D, true);
    drivers->leds.set(tap::gpio::Leds::E, true);
    drivers->leds.set(tap::gpio::Leds::F, true);
    drivers->leds.set(tap::gpio::Leds::G, true);
    drivers->leds.set(tap::gpio::Leds::H, true);

#ifdef PLATFORM_HOSTED
    tap::motorsim::SimHandler::resetMotorSims();
    // Blocking call, waits until Windows Simulator connects.
    tap::communication::TCPServer::MainServer()->getConnection();
#endif

    

    while (1)
    {
        // do this as fast as you can
        PROFILE(drivers->profiler, updateIo, (drivers));
        if (sendMotorTimeout.execute())
        {
            PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
        }
        drivers->leds.set(tap::gpio::Leds::A, !drivers->vtm.keyPressed(tap::communication::serial::Vtm::Key::A));
        drivers->canRxHandler.pollCanData();

        encoder_msg.data = rf_motor.getEncoderWrapped();
        pub_encoder.publish(&encoder_msg);
		nh.spinOnce();
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
