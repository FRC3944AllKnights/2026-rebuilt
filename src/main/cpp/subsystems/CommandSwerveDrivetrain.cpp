#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc/RobotController.h>

using namespace subsystems;

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

void CommandSwerveDrivetrain::DriveDefaultCommand(units::velocity::meters_per_second_t inputLeftY,
        units::velocity::meters_per_second_t inputLeftX,
        units::angular_velocity::radians_per_second_t inputRightX,
        swerve::requests::FieldCentric drive) {
    // If given zero-velocity command, set X-brake formation
    // Else: drive normally
    if (inputLeftY == 0_mps && inputLeftX == 0_mps && inputRightX == 0_rad_per_s) {
        this->ApplyRequest([this] {return brake;});
    }
    else {
        drive.WithVelocityX(inputLeftY) // Drive forward with negative Y (forward)
            .WithVelocityY(inputLeftX) // Drive left with negative X (left)
            .WithRotationalRate(inputRightX); // Drive counterclockwise with negative X (left)
    }
}
