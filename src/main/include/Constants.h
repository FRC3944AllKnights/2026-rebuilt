#pragma once

#include <units/dimensionless.h>
#include <string>

namespace AprilTagConstants {
    // All 32 AprilTag IDs for the 2026 Rebuilt & Welded field.
    // Heights and field positions come from frc::AprilTagFieldLayout at runtime.
    // Convention: odd IDs = centered on face, even IDs = offset.

    // ── Red Hub (central hex structure) ──
    constexpr int kRedHubNorthCenter   = 5;
    constexpr int kRedHubNorthOffset   = 8;
    constexpr int kRedHubSouthCenter   = 11;
    constexpr int kRedHubSouthOffset   = 2;
    constexpr int kRedHubWestCenter    = 9;
    constexpr int kRedHubWestOffset    = 10;
    constexpr int kRedHubEastCenter    = 3;
    constexpr int kRedHubEastOffset    = 4;

    // ── Blue Hub ──
    constexpr int kBlueHubNorthCenter  = 27;
    constexpr int kBlueHubNorthOffset  = 18;
    constexpr int kBlueHubSouthCenter  = 21;
    constexpr int kBlueHubSouthOffset  = 24;
    constexpr int kBlueHubWestCenter   = 19;
    constexpr int kBlueHubWestOffset   = 20;
    constexpr int kBlueHubEastCenter   = 25;
    constexpr int kBlueHubEastOffset   = 26;

    // ── Red Trench (above/below hub) ──
    constexpr int kRedTrenchNorthCenter  = 7;
    constexpr int kRedTrenchNorthOffset  = 6;
    constexpr int kRedTrenchSouthCenter  = 1;
    constexpr int kRedTrenchSouthOffset  = 12;

    // ── Blue Trench ──
    constexpr int kBlueTrenchNorthCenter = 17;
    constexpr int kBlueTrenchNorthOffset = 28;
    constexpr int kBlueTrenchSouthCenter = 23;
    constexpr int kBlueTrenchSouthOffset = 22;

    // ── Red Outpost ──
    constexpr int kRedOutpostCenter  = 13;
    constexpr int kRedOutpostOffset  = 14;

    // ── Blue Outpost ──
    constexpr int kBlueOutpostCenter = 29;
    constexpr int kBlueOutpostOffset = 30;

    // ── Red Tower ──
    constexpr int kRedTowerCenter  = 15;
    constexpr int kRedTowerOffset  = 16;

    // ── Blue Tower ──
    constexpr int kBlueTowerCenter = 31;
    constexpr int kBlueTowerOffset = 32;

    // Convenience arrays
    constexpr int kRedShooterTags[]  = { 9, 10 };
    constexpr int kBlueShooterTags[] = { 25, 26 };
}

namespace ClimberConstants {

}

namespace DrivetrainConstants {

}

namespace IntakeConstants {

}

namespace VisionConstants {
    const std::string kLimelightName = "limelight-intake";
}

namespace ShooterConstants {

    // Hard dimensions

    const double shooterCircumference = 12.56; // inches
    const double exitAngle = 45.0; // degrees
    const double shooterHeight = 20.0; // inches, measured from ball center at exit
    const double shooterOffsetFromRearBumper = 10.0; // inches
    const double forwardCameraHeight = 18.0; // inches
    const double forwardCameraAngle = 0.0; // degrees (mounted level)
    const double forwardCameraShooterOffset = 6.0; // inches - distance between camera and ball exit
    
    const double hubHeight = 72.0; // inches
    const double hubTagHeight = 44.25; // inches
    const double hubWidthMax = 44.4; // inches (in x direction - parallel to driver view)

    const double g = 386.2205; // inches per second^2 - gravitational acceleration on Earth surface

    // Empirical (not yet measured as of Feb 3 2026)

    const double shooterBallSpeedTransferPercent = 1.0;
    const double targetShooterSpeedBase = 500; // rotations per minute - 6 ft range

    // Selected

    const double baseRange = 136.46; // inches - tower to hub
    const double minimumRange = 96.0 ; // inches - enough to clear front lip of hub at 60 deg shooter
    const int LEDPort = 1;
    const int kLength = 60; // number of LEDs in the LED strip

    // PID

    const units::dimensionless_t shooterP = 0.1;
    const units::dimensionless_t shooterI = 0.0;
    const units::dimensionless_t shooterD = 0.0;

    const units::dimensionless_t indexerP = 0.1;
    const units::dimensionless_t indexerI = 0.0;
    const units::dimensionless_t indexerD = 0.0;

    // Mode

    const bool debugPrintsEnabled = true;
    
}

namespace CANConstants {

    // CAN Bus

    const std::string kCANBusName = "rio";

    // Robot Core

    const int kPDPId = 0;
    const int kRIOId = 1;

    // Drivetrain Motors

    const int kFrontLeftDriveMotorId = 11;
    const int kBackLeftDriveMotorId = 12;
    const int kFrontRightDriveMotorId = 13;
    const int kBackRightDriveMotorId = 14;

    const int kFrontLeftSteerMotorId = 15;
    const int kBackLeftSteerMotorId = 16;
    const int kFrontRightSteerMotorId = 17;
    const int kBackRightSteerMotorId = 18;

    // Drivetrain Sensors

    const int kPigeonIMUId = 20;
    const int kFrontLeftEncoderId = 21;
    const int kBackLeftEncoderId = 22;
    const int kFrontRightEncoderId = 23;
    const int kBackRightEncoderId = 24;

    // Intake

    const int kIntakeDeployLeftMotorId = 30;
    const int kIntakeDeployRightMotorId = 31;
    const int kIntakeRollerMotorId = 32;

    // Shooter

    const int kShooterLeftMotorId = 40;
    const int kShooterRightMotorId = 41;
    const int kIndexerMotorId = 42;

    // Climber

    const int kClimberMotorId = 50;
}