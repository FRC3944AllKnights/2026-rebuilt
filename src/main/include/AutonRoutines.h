#pragma once

#include <frc2/command/CommandPtr.h>
#include <string>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class AutonRoutines {
    public:
        AutonRoutines(
            subsystems::CommandSwerveDrivetrain& drivetrain,
            swerve::requests::FieldCentric& drive,
            subsystems::IntakeSubsystem& intake,
            subsystems::ShooterSubsystem& shooter,
            subsystems::VisionSubsystem& vision
        );

        frc2::CommandPtr DoNothing();
        frc2::CommandPtr DriveForward();

        static constexpr std::string_view kDoNothing = "Do Nothing";
        static constexpr std::string_view kDriveForward = "Drive Forward";

    private:

    subsystems::CommandSwerveDrivetrain& m_drivetrain;
    swerve::requests::FieldCentric& m_drive;
    subsystems::IntakeSubsystem& m_intake;
    subsystems::ShooterSubsystem& m_shooter;
    subsystems::VisionSubsystem& m_vision;
};