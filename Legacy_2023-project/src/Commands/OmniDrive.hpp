#ifndef OMNIDRIVE
#define OMNIDRIVE


#include "tap/control/command.hpp"
#include "../Subsystems/DriveTrain.hpp"
#include "drivers.hpp"

namespace src::DriveTrain
{
class OmniDrive : public tap::control::Command
{
    public:

        OmniDrive(src::Drivers* drivers, DriveTrainSubsystem* drive_train);

        void initialize() override;

        void execute() override;

        void end(bool interrupted) override;

        bool isReady() override;

        bool isFinished() const override;

        const char* getName() const {return "OmniDrive Command";}

    private:

        DriveTrainSubsystem* drive_train;
        src::Drivers* drivers;
        bool aSet;
};
}


#endif /* OMNIDRIVE */