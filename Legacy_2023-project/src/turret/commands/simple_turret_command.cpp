#include "simple_turret_command.hpp"

namespace src::turret
{

SimpleTurretCommand::SimpleTurretCommand(src::Drivers* drivers, TurretSubsystem* turret) : 
drivers(drivers), turret(turret)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(turret));
}

void SimpleTurretCommand::initialize()
{
}

void SimpleTurretCommand::execute()
{
    float_t pitch = turret->getPitch(), yaw = turret->getYaw();
    pitch = 5000 * (drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::R) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::F));
    yaw = 5000 * (drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::V) - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::C));
    turret->setDesiredOutput(pitch, yaw);
}

void SimpleTurretCommand::end(bool)
{
}

bool SimpleTurretCommand::isFinished() const
{
    return false;
}

bool SimpleTurretCommand::isReady()
{
    return true;
}

}