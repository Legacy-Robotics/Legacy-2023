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
    drivers->refSerial.resetKeys();
}

void SimpleSwerveCommand::execute()
{
    float_t x = 0, y = 0 , rot = 0;
    y = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::W) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::S);
    x = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::A) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::D);
    rot = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::Q) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::E);
    chassis->setDesiredOutput(x, y, rot);
}

void SimpleSwerveCommand::end(bool)
{
    chassis->setDesiredOutput(0, 0, 0);
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