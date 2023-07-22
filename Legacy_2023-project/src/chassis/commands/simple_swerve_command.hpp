#pragma once

#include "tap/control/command.hpp"
#include "../subsystems/chassis.hpp"
#include "drivers.hpp"

namespace src::chassis
{
class SimpleSwerveCommand : public tap::control::Command
{
    public:

        SimpleSwerveCommand(src::Drivers* drivers, ChassisSubsystem* chassis);

        void initialize() override;

        void execute() override;

        void end(bool interrupted) override;

        bool isReady() override;

        bool isFinished() const override;

        const char* getName() const {return "Drivetrain Drive Command";}

    private:

        ChassisSubsystem* chassis;
        src::Drivers* drivers;
        bool safety;
};
}
