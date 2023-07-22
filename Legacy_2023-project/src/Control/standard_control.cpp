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
#include "../chassis/subsystems/chassis.hpp"
//include commands below
#include "../chassis/commands/simple_swerve_command.hpp"

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;

//include subsystem namespaces below
using namespace src::chassis;
using namespace src::turret;

namespace StandardControl {

// Define Subsystems Here
ChassisSubsystem chassis(drivers());
TurretSubsystem turret(drivers());

// Define Commands Here
SimpleSwerveCommand swerve(drivers(), &chassis);
SimpleTurretCommand turretControl(drivers(), &turret);

// Define Command Mappings Here
// Register Subsystems with drivers->commandScheduler.registerSubsystem(&subsystem_name);
void registerSubsystems(src::Drivers* drivers)
{
    drivers->commandScheduler.registerSubsystem(&chassis);
}

// Initialize Subsystems with subsystem.initialize();
void initializeSubsystems()
{
    drivetrain.initialize();
}

// Set Default Command with subsystem.setDefaultCommand(&command)
void setDefaultCommands(src::Drivers* drivers)
{
    drivetrain.setDefaultCommand(&swerve);
}

// Set Commands scheduled on startup
void startupCommands(src::Drivers* drivers)
{

}

// Register IO Mappings with drivers->commandMapper.addMap(&commandMapping)
void registerIOMappings(src::Drivers* drivers)
{
}

} //namespace StandardControl

namespace src::Control
{
    void initializeSubsystemCommands(src::Drivers* drivers)
    {
        StandardControl::initializeSubsystems();
        StandardControl::registerSubsystems(drivers);
        StandardControl::setDefaultCommands(drivers);
        StandardControl::startupCommands(drivers);
        StandardControl::registerIOMappings(drivers);
    }
} //namespace src::Control

#endif