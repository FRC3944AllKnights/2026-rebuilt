#include "generated/TunerConstants.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/IntakeSubsystem.h"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}

subsystems::IntakeSubsystem TunerConstants::CreateIntake()
{
    return subsystems::IntakeSubsystem();
}
