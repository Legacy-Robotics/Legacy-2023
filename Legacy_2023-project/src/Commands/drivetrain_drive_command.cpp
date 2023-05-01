#include "drivetrain_drive_command.hpp"
#

namespace src::Drivetrain
{

OmniDriveCommand::OmniDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* dt) : 
drivers(drivers), dt(dt)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(dt));
}

void OmniDriveCommand::initialize()
{
    aSet = false;
}

void OmniDriveCommand::execute()
{
    aSet = true;
    float_t y = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::W) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::S);
    float_t x = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::A) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::D);
    float_t rot = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::Q) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::E);
    
    dt->setDesiredOutput(x, y, rot);
}

void OmniDriveCommand::end(bool)
{
    dt->setDesiredOutput(0, 0, 0);
    aSet = false;
}

bool OmniDriveCommand::isFinished() const
{
    return false;
}

bool OmniDriveCommand::isReady()
{
    return true;
}

}