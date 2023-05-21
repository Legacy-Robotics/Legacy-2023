#ifndef TURRET_MOVE_COMMAND
#define TURRET_MOVE_COMMAND

#include "tap/control/command.hpp"
#include "../Subsystems/Turret.hpp"
#include "drivers.hpp"

namespace src::Turret
{
class TurretMoveCommand : public tap::control::Command
{
    public:

        TurretMoveCommand(src::Drivers* drivers, TurretSubsystem* t);

        void initialize() override;

        void execute() override;

        void end(bool interrupted) override;

        bool isReady() override;

        bool isFinished() const override;

        const char* getName() const {return "Turret Move Command";}

    private:

        TurretSubsystem* t;
        src::Drivers* drivers;
        bool aSet;
};
}

#endif