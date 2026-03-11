// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <string>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "Telemetry.h"
#include <iostream>
#include <cmath>

class RobotContainer {
private:
    units::meters_per_second_t MaxSpeed = 1.0 * TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};

    /* Snap-to-45 heading lock request */
    swerve::requests::FieldCentricFacingAngle facingAngle = swerve::requests::FieldCentricFacingAngle{}
        .WithDeadband(MaxSpeed * 0.1)
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage);
    units::degree_t m_snapHeading{0_deg};

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};

    frc2::CommandXboxController joystick{0};

public:
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};
    subsystems::IntakeSubsystem intake{TunerConstants::CreateIntake()};
    subsystems::ShooterSubsystem shooter{TunerConstants::CreateShooter()};
    subsystems::ClimberSubsystem climber{TunerConstants::CreateClimber()};
    subsystems::VisionSubsystem vision{TunerConstants::CreateVision()};

    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

    // Auto mode string constants
    static constexpr std::string_view kDoNothing = "Do Nothing";
    static constexpr std::string_view kDriveForward = "Drive Forward";

private:
    void ConfigureBindings();
    subsystems::VisionSubsystem::visionTarget hubVisionTarget;

    frc::SendableChooser<std::string> m_autoChooser;
};
