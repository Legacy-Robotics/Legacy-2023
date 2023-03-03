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

#include "tap/board/board.hpp"

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

/* define timers here -------------------------------------------------------*/
tap::arch::PeriodicMilliTimer sendMotorTimeout(2);

// Place any sort of input/output initialization here. For example, place
// serial init stuff here.
static void initializeIo(src::Drivers *drivers);

// Anything that you would like to be called place here. It will be called
// very frequently. Use PeriodicMilliTimers if you don't want something to be
// called as frequently.
static void updateIo(src::Drivers *drivers);

static constexpr tap::algorithms::SmoothPidConfig MY_PID_CONFIG = {
    .kp = 9.0f,
    .ki = 0.0f,
    .kd = 0.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 3'000.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.1f,
};

static constexpr tap::motor::MotorId MOTOR_ID = tap::motor::MOTOR2;
static constexpr tap::can::CanBus CAN_BUS = tap::can::CanBus::CAN_BUS1;
static constexpr int DESIRED_RPM = 3000;

    
tap::algorithms::SmoothPid pidController(MY_PID_CONFIG);
tap::motor::DjiMotor motor(src::DoNotUse_getDrivers(), MOTOR_ID, CAN_BUS, false, "cool motor");

int main()
{
#ifdef PLATFORM_HOSTED
    std::cout << "Simulation starting..." << std::endl;
#endif

    /*
     * NOTE: We are using DoNotUse_getDrivers here because in the main
     *      robot loop we must access the singleton drivers to update
     *      IO states and run the scheduler.
     */
    src::Drivers *drivers = src::DoNotUse_getDrivers();
    drivers->can.initialize();

    Board::initialize();
    initializeIo(drivers);

    motor.initialize();

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
            // PROFILE(drivers->profiler, drivers->mpu6500.periodicIMUUpdate, ());
            // PROFILE(drivers->profiler, drivers->commandScheduler.run, ());
            // PROFILE(drivers->profiler, drivers->djiMotorTxHandler.encodeAndSendCanData, ());
            // PROFILE(drivers->profiler, drivers->terminalSerial.update, ());
            pidController.runControllerDerivateError(DESIRED_RPM - motor.getShaftRPM(), 1);
            motor.setDesiredOutput(static_cast<int32_t>(pidController.getOutput()));
            drivers->djiMotorTxHandler.encodeAndSendCanData();
        }
        drivers->canRxHandler.pollCanData();
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
    drivers->mpu6500.init();
    drivers->refSerial.initialize();
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
    drivers->remote.read();
    drivers->mpu6500.read();
}
