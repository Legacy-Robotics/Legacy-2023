#include "turret_move_command.hpp"

namespace src::Turret
{

TurretMoveCommand::TurretMoveCommand(src::Drivers* drivers, TurretSubsystem* t) : 
drivers(drivers), t(t)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(t));
}

void TurretMoveCommand::initialize()
{
    aSet = false;
    drivers->refSerial.resetKeys();
}

void TurretMoveCommand::execute()
{
    aSet = true;
    float_t yaw = 0, pitch = 0;
    if (!drivers->refSerial.controlIsDisabled()) {
        yaw = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::Q)
            - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::E);
        
        pitch = drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::R)
            - drivers->refSerial.getKey(tap::communication::serial::RefSerialData::Rx::Key::F);
   
       
    }
    t->setDesiredOutput(yaw, pitch);
}

void TurretMoveCommand::end(bool)
{
    t->setDesiredOutput(0, 0);
    aSet = false;
}

bool TurretMoveCommand::isFinished() const
{
    return false;
}

bool TurretMoveCommand::isReady()
{
    return true;
}

}