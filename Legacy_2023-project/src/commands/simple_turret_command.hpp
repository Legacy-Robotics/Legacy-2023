#pragma once
#include "tap/control/command.hpp"
#include "../subsystems/turret.hpp"
#include "drivers.hpp"

namespace src::turret
{
class SimpleTurretCommand : public tap::control::Command
{
    public:

        SimpleTurretCommand(src::Drivers* drivers, TurretSubsystem* turret);

        void initialize() override;

        void execute() override;

        void end(bool interrupted) override;

        bool isReady() override;

        bool isFinished() const override;

        const char* getName() const {return "Drivetrain Drive Command";}

    private:

        TurretSubsystem* turret;
        src::Drivers* drivers;
        bool safety;
};
}