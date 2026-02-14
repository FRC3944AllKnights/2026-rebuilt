// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

RobotContainer::RobotContainer()
{
    ConfigureBindings();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        // If joystick is idle, set X formation
        // Else: drive normally
        frc2::cmd::Run([this] { drivetrain.DriveDefaultCommand(
            -joystick.GetLeftY() * MaxSpeed, // Drive forward with negative Y (forward)
            -joystick.GetLeftX() * MaxSpeed, // Drive left with negative X (left)
            -joystick.GetRightX() * MaxAngularRate, // Drive counterclockwise with negative X (left)
            drive);})
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    // Intake controls

    joystick.A().WhileTrue(frc2::cmd::Run([this] { intake.RunIntake(1.0); }))
        .OnFalse(frc2::cmd::Run([this] { intake.RunIntake(0.0); }));
    joystick.B().WhileTrue(frc2::cmd::Run([this] { intake.RunIntake(-1.0); }))
        .OnFalse(frc2::cmd::Run([this] { intake.RunIntake(0.0); }));
    joystick.X().OnTrue(frc2::cmd::Run([this] { intake.SetIntakePosition(false); }));
    joystick.Y().OnTrue(frc2::cmd::Run([this] { intake.SetIntakePosition(true); }));

    // Shooter controls

    joystick.LeftTrigger().WhileTrue(frc2::cmd::Run([this] {
            double speed = joystick.GetLeftTriggerAxis();
            shooter.SpinUpShooter(speed);
        }))
        .WhileFalse(frc2::cmd::Run([this] {
            shooter.SpinUpShooter(0.0);
        }));
    joystick.RightTrigger().WhileTrue(frc2::cmd::Run([this]
        { shooter.SetIndexerSpeed(joystick.GetRightTriggerAxis()); }))
        .WhileFalse(frc2::cmd::Run([this] { shooter.SetIndexerSpeed(0.0); }));

    // Climber controls

    joystick.LeftBumper().WhileTrue(frc2::cmd::Run([this] { climber.SetClimberPosition(-1.0); }))
        .OnFalse(frc2::cmd::Run([this] { climber.SetClimberPosition(0.0); }));
    joystick.RightBumper().WhileTrue(frc2::cmd::Run([this] { climber.SetClimberPosition(1.0); }))
        .OnFalse(frc2::cmd::Run([this] { climber.SetClimberPosition(0.0); }));

    // Auto-align controls

    // Align with hub (target) and power up shooter to appropriate speed
    /*
    joystick.POVUp().WhileTrue(
        frc2::cmd::Sequence(
            drivetrain.ApplyRequest([this]() -> auto&& {
                return drive.WithVelocityX(0_mps) // Drive forward with negative Y (forward)
                    .WithVelocityY(0_mps) // Drive left with negative X (left)
                    .WithRotationalRate(units::angular_velocity::radians_per_second_t{0}); // Drive counterclockwise with negative X (left)
            }),
            frc2::cmd::Run([this] { shooter.SpinUpShooter(true); })
        )
    )
    .OnFalse(
        frc2::cmd::Sequence(
            // Get vision target data
            frc2::cmd::Run([this] { vision.setVisionTarget(hubVisionTarget); }),
            // Set drivetrain angle - TODO: Implement actual alignment logic
            frc2::cmd::Run([this] { drivetrain.DriveDefaultCommand(0_mps, 0_mps, 0_rad_per_s, drive); }),
            // Set shooter speed
            frc2::cmd::Run([this] { shooter.SpinUpShooter(shooter.getTargetShooterRPM(hubVisionTarget.range)); })
        )
    );

    // Align with tower: Snap robot angle to 90 degrees or -90 degrees
    joystick.POVDown().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
                return drive.WithVelocityX(0_mps) // Drive forward with negative Y (forward)
                    .WithVelocityY(0_mps) // Drive left with negative X (left)
                    .WithRotationalRate(units::angular_velocity::radians_per_second_t{0}); // Drive counterclockwise with negative X (left)
            })
    )
    .OnFalse(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(0_mps) // Drive forward with negative Y (forward)
                .WithVelocityY(0_mps) // Drive left with negative X (left)
                .WithRotationalRate(units::angular_velocity::radians_per_second_t{0}); // Drive counterclockwise with negative X (left)
        })
    );
    */

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    }

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    // Simple drive forward auton
    return frc2::cmd::Sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(frc::Rotation2d{0_deg}); }),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(0.5_mps)
                .WithVelocityY(0_mps)
                .WithRotationalRate(0_tps);
        })
        .WithTimeout(5_s),
        // Finally idle for the rest of auton
        drivetrain.ApplyRequest([] { return swerve::requests::Idle{}; })
    );
}
