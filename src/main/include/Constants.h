#pragma once

namespace ClimberConstants {

}

namespace DrivetrainConstants {

}

namespace IntakeConstants {

}

namespace ShooterConstants {

    // Hard dimensions

    const double shooterCircumference = 12.56; // inches
    const double exitAngle = 45.0; // degrees
    const double shooterHeight = 20.0; // inches, measured from ball center at exit
    const double shooterOffsetFromRearBumper = 10.0; // inches
    const double forwardCameraHeight = 18.0; // inches
    const double forwardCameraAngle = 45.0; // degrees
    const double forwardCameraShooterOffset = 6.0; // inches - distance between camera and ball exit
    
    const double hubHeight = 72.0; // inches
    const double hubTagHeight = 44.25; // inches
    const double hubWidthMax = 44.4; // inches (in x direction - parallel to driver view)
    const int aprilTagIDHubRed = 9; // ID 10 is offset
    const int aprilTagIDHubBlue = 25; // ID 26 is offset

    const double g = 386.2205; // inches per second^2 - gravitational acceleration on Earth surface

    // Empirical (not yet measured as of Feb 3 2026)

    const double shooterBallSpeedTransferPercent = 1.0;
    const double targetShooterSpeedBase = 500; // rotations per minute - 6 ft range

    // Selected

    const double baseRange = 136.46; // inches - tower to hub
    const double minimumRange = 96.0 ; // inches - enough to clear front lip of hub at 60 deg shooter
    
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