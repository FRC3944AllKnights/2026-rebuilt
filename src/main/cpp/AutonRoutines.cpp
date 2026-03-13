#include "AutonRoutines.h"

#include <frc2/command/Commands.h>

AutonRoutines::AutonRoutines(
    subsystems::CommandSwerveDrivetrain& drivetrain,
    swerve::requests::FieldCentric& drive,
    subsystems::IntakeSubsystem& intake,
    subsystems::ShooterSubsystem& shooter,
    subsystems::VisionSubsystem& vision)
    : m_drivetrain{drivetrain},
      m_drive{drive},
      m_intake{intake},
      m_shooter{shooter},
      m_vision{vision}
      {}

frc2::CommandPtr AutonRoutines::DoNothing() {
    return frc2::cmd::None();
}

frc2::CommandPtr AutonRoutines::DriveForward() {
    return frc2::cmd::Sequence(
            m_drivetrain.RunOnce([this] { m_drivetrain.SeedFieldCentric(frc::Rotation2d{0_deg}); }),
            m_drivetrain.ApplyRequest([this]() -> auto&& {
                return m_drive.WithVelocityX(0.5_mps)
                    .WithVelocityY(0_mps)
                    .WithRotationalRate(0_tps);
            })
            .WithTimeout(5_s),
            m_drivetrain.ApplyRequest([] { return swerve::requests::Idle{}; })
        );
}