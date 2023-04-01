#include "StandardControl.hpp"
#ifdef TARGET_STANDARD
/*
 * NOTE: We are using the DoNotUse_getDrivers() function here
 *      because this file defines all subsystems and command
 *      and thus we must pass in the single statically allocated
 *      Drivers class to all of these objects.
 */


using namespace tap;
using namespace tap::control;
using namespace tap::communication::serial;

//include subsystem namespaces below
using namespace src::DriveTrain;

namespace StandardControl {
src::Drivers *drivers = src::DoNotUse_getDrivers();

// Define Subsystems Here
src::DriveTrain::DriveTrainSubsystem drive_train(drivers);
// Define Commands Here
src::DriveTrain::OmniDrive omni_drive(drivers, &drive_train);

// Define Command Mappings Here
/*
HoldCommandMapping leftSwitchUp(
    drivers(), 
    {&omni_drive}, 
    RemoteMapState(Remote::Switch::LEFT_SWITCH, Remote::SwitchState::UP));
*/

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
    //drivers->commandMapper.addMap(&leftSwitchUp);
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