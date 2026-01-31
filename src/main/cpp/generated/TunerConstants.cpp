#include "generated/TunerConstants.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ClimberSubsystem.h"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain()
{
    return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}

subsystems::IntakeSubsystem TunerConstants::CreateIntake()
{
    return subsystems::IntakeSubsystem();
}

subsystems::ShooterSubsystem TunerConstants::CreateShooter()
{
    return subsystems::ShooterSubsystem();
}

subsystems::ClimberSubsystem TunerConstants::CreateClimber()
{
    return subsystems::ClimberSubsystem();
}
