#include "drivetrain_drive_command.hpp"

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
    drivers->refSerial.resetKeys();
}

void OmniDriveCommand::execute()
{
    aSet = true;
    float_t x = 0, y = 0 , rot = 0;
    if (!drivers->refSerial.controlIsDisabled()) {
        y = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::W) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::S);
        x = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::A) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::D);
        rot = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::Q) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::E);
    }
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