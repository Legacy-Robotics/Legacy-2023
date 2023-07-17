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
    safety = true;
    drivers->refSerial.resetKeys();
}

void SimpleTurretCommand::execute()
{
    if (!drivers->refSerial.controlIsDisabled()) {
    }
}

void SimpleTurretCommand::end(bool)
{
    safety = true;
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