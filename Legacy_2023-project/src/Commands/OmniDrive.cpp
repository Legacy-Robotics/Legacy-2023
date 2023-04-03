#include "OmniDrive.hpp"

namespace src::DriveTrain
{
    OmniDrive::OmniDrive(src::Drivers* drivers, DriveTrainSubsystem* drive_train) : drivers(drivers), drive_train(drive_train) {
        addSubsystemRequirement(drive_train);
    }

    void OmniDrive::initialize()
    {
        
    }

    void OmniDrive::execute() {}

    void OmniDrive::end(bool interrupted) {}

    bool OmniDrive::isReady() { return false; }

    bool OmniDrive::isFinished() const { return false; }
} 