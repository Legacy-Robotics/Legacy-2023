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