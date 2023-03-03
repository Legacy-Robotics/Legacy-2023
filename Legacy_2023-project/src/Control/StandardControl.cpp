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
#include "../Subsystems/DriveTrain.hpp"
//include commands below
#include "../Commands/OmniDrive.hpp"

/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */
src::driversFunc drivers = src::DoNotUse_getDrivers;

using namespace tap;
using namespace tap::control;
using namespace tap::communication::serial;

//include subsystem namespaces below
using namespace src::DriveTrain;

namespace StandardControl {

// Define Subsystems Here
src::DriveTrain::DriveTrainSubsystem drivet_train(drivers());
// Define Commands Here
OmniDrive omni_drive(drivers(), &drive_train);

// Define Command Mappings Here
HoldCommandMapping leftSwitchUp(
    drivers(), 
    {&omni_drive}, 
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
// Register Subsystems with drivers->commandScheduler.registerSubsystem(&subsystem_name);
void registerSubsystems(src::Drivers* drivers)
{
    drivers->commandScheduler.registerSubsystem(&drive_train);
}

// Initialize Subsystems with subsystem.initialize();
void initializeSubsystems()
{
    drive_train.initialize();
}

// Set Default Command with subsystem.setDefaultCommand(&command)
void setDefaultCommands(src::Drivers* drivers)
{
    drive_train.setDefaultCommand(&omni_drive);
}

// Set Commands scheduled on startup
void startupCommands(src::Drivers* drivers)
{

}

// Register IO Mappings with drivers->commandMapper.addMap(&commandMapping)
void registerIOMappings(src::Drivers* drivers)
{
    drivers->commandMapper.addMap(&leftSwitchUp);
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

#endif /*TARGET_STANDARD*/