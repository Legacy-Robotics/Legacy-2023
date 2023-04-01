#ifdef TARGET_STANDARD
#ifndef STANDARD_CONTROL_HPP_
#define STANDARD_CONTROL_HPP_
#include "drivers.hpp"
#include "drivers_singleton.hpp"

#include "tap/control/command_mapper.hpp"
#include "tap/control/hold_command_mapping.hpp"
#include "tap/control/hold_repeat_command_mapping.hpp"
#include "tap/control/press_command_mapping.hpp"
#include "tap/control/setpoint/commands/calibrate_command.hpp"
#include "tap/control/toggle_command_mapping.hpp"
//include subsystems below
#include "../Subsystems/DriveTrain.hpp"
//include commands below
#include "../Commands/OmniDrive.hpp"

namespace src::Control
{
void initializeSubsystemCommands(src::Drivers* drivers);
}

#endif /*STANDARD_CONTROL_HPP_*/
#endif /*TARGET_STANDARD*/