#ifndef DRIVETRAIN_DRIVE_COMMAND
#define DRIVETRAIN_DRIVE_COMMAND

#include "tap/control/command.hpp"
#include "../Subsystems/drivetrain.hpp"
#include "drivers.hpp"

namespace src::Drivetrain
{
class OmniDriveCommand : public tap::control::Command
{
    public:

        OmniDriveCommand(src::Drivers* drivers, DrivetrainSubsystem* dt);

        void initialize() override;

        void execute() override;

        void end(bool interrupted) override;

        bool isReady() override;

        bool isFinished() const override;

        const char* getName() const {return "Drivetrain Drive Command";}

    private:

        DrivetrainSubsystem* dt;
        src::Drivers* drivers;
        bool aSet;
};
}

#endif