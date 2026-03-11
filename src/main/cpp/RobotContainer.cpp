// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer()
{
    facingAngle.HeadingController.SetPID(3, 0, 0.1);
    facingAngle.HeadingController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
    // Wire vision subsystem to drivetrain for MegaTag2 + Kalman filter fusion
    vision.SetDrivetrain(&drivetrain);

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
            -frc::ApplyDeadband(joystick.GetLeftY(), 0.1) * MaxSpeed, // Drive forward with negative Y (forward)
            -frc::ApplyDeadband(joystick.GetLeftX(), 0.1) * MaxSpeed, // Drive left with negative X (left)
            -frc::ApplyDeadband(joystick.GetRightX(), 0.1) * MaxAngularRate, // Drive counterclockwise with negative X (left)
            drive);}, {&drivetrain})
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    joystick.A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    joystick.B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-joystick.GetLeftY(), -joystick.GetLeftX()});
    }));

    // Shooter controls
    shooter.SetVision(&vision);

    shooter.SetDefaultCommand(frc2::cmd::Run([this] {
        shooter.SpinUpShooter(0.0);
        }, {&shooter}));
    joystick.Back().OnTrue(frc2::cmd::RunOnce([this] {
        shooter.ToggleAdjustableRPM();
        frc::SmartDashboard::PutBoolean("Shooter Adjustable RPM Enabled", shooter.IsAdjustableRPMEnabled());
    }));
    joystick.LeftTrigger().WhileTrue(frc2::cmd::Run([this] {
            std::cout << "Left Trigger Is Pressed" << std::endl;
            double speed = joystick.GetLeftTriggerAxis();
            shooter.SpinUpShooter(speed);
        }, {&shooter}));
    
    joystick.RightTrigger().WhileTrue(frc2::cmd::Run([this] {
        std::cout << "Right Trigger Is Pressed" << std::endl;
        shooter.SetIndexerSpeed(joystick.GetRightTriggerAxis()); }))
        .WhileFalse(frc2::cmd::Run([this] { shooter.SetIndexerSpeed(0.0); }, {&shooter}));

    // Snap-to-45: Right stick button locks heading to nearest 45° while allowing translation
    joystick.RightStick().OnTrue(
        frc2::cmd::RunOnce([this] {
            double heading = drivetrain.GetState().Pose.Rotation().Degrees().value();
            m_snapHeading = units::degree_t{std::round(heading / 45.0) * 45.0};
            facingAngle.HeadingController.Reset();
        })
    );
    joystick.RightStick().WhileTrue(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return facingAngle
                .WithVelocityX(-frc::ApplyDeadband(joystick.GetLeftY(), 0.1) * MaxSpeed)
                .WithVelocityY(-frc::ApplyDeadband(joystick.GetLeftX(), 0.1) * MaxSpeed)
                .WithTargetDirection(frc::Rotation2d{m_snapHeading});
        })
    );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (joystick.Back() && joystick.Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (joystick.Back() && joystick.X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (joystick.Start() && joystick.Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (joystick.Start() && joystick.X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
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
