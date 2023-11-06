#ifdef TARGET_STANDARD

#include "drivers.hpp"
#include "drivers_singleton.hpp"
//
#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//include subsystems below
#include "../subsystems/chassis.hpp"
//include commands below
#include "../commands/simple_swerve_command.hpp"

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers_ = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;

//include subsystem namespaces below
using namespace src::chassis;

namespace StandardControl {

// Define Subsystems Here

// Define Command Mappings Here
// Register Subsystems with drivers->commandScheduler.registerSubsystem(&subsystem_name);
void registerSubsystems(src::Drivers* drivers_)
{
}

// Initialize Subsystems with subsystem.initialize();
void initializeSubsystems()
{
}

// Set Default Command with subsystem.setDefaultCommand(&command)
void setDefaultCommands(src::Drivers* drivers_)
{
}

// Set Commands scheduled on startup
void startupCommands(src::Drivers* drivers_)
{

}

// Register IO Mappings with drivers->commandMapper.addMap(&commandMapping)
void registerIOMappings(src::Drivers* drivers_)
{
}

} //namespace StandardControl

namespace src::Control
{
    void initializeSubsystemCommands(src::Drivers* drivers_)
    {
        StandardControl::initializeSubsystems();
        StandardControl::registerSubsystems(drivers_);
        StandardControl::setDefaultCommands(drivers_);
        StandardControl::startupCommands(drivers_);
        StandardControl::registerIOMappings(drivers_);
    }
} //namespace src::Control

#endif