#include "simple_swerve_command.hpp"


using namespace tap::communication::serial;
namespace src::chassis
{

SimpleSwerveCommand::SimpleSwerveCommand(src::Drivers* drivers, ChassisSubsystem* chassis) : 
drivers(drivers), chassis(chassis)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void SimpleSwerveCommand::initialize()
{
}

void SimpleSwerveCommand::execute()
{
    float_t x = 0, y = 0 , rot = 0;
    y = drivers->vtm.getKey(Vtm::Key::W) - drivers->vtm.getKey(Vtm::Key::S);
    x = drivers->vtm.getKey(Vtm::Key::A) - drivers->vtm.getKey(Vtm::Key::D);
    rot = drivers->vtm.getKey(Vtm::Key::Q) - drivers->vtm.getKey(Vtm::Key::E);
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