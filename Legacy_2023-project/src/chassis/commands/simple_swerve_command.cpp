#include "simple_swerve_command.hpp"

namespace src::chassis
{

SimpleSwerveCommand::SimpleSwerveCommand(src::Drivers* drivers, ChassisSubsystem* chassis) : 
drivers(drivers), chassis(chassis)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void SimpleSwerveCommand::initialize()
{
    safety = false;
    drivers->refSerial.resetKeys();
}

void SimpleSwerveCommand::execute()
{
    safety = true;
    float_t x = 0, y = 0 , rot = 0;
    if (!drivers->refSerial.controlIsDisabled()) {
        y = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::W) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::S);
        x = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::A) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::D);
        rot = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::Q) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::E);
    }
    chassis->setDesiredOutput(x, y, rot);
}

void SimpleSwerveCommand::end(bool)
{
    chassis->setDesiredOutput(0, 0, 0);
    safety = false;
}

bool SimpleSwerveCommand::isFinished() const
{
    return false;
}

bool SimpleSwerveCommand::isReady()
{
    return true;
}

}