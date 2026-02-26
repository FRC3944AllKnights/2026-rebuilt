#pragma once
#include <units/angle.h>
#include <units/current.h>
#include <units/angular_jerk.h>

#include <units/dimensionless.h>

namespace ClimberConstants {

}

namespace DrivetrainConstants {

}

namespace IntakeConstants {

    // Gear ratio between motor and arm output shaft
    const double intakeDeployGearRatio = 12.8;

    // --- PID Gains (Slot 0) for deploy position control ---
    const double intakeDeployP = 0.1;
    const double intakeDeployI = 0.0;
    const double intakeDeployD = 0.0;

    // --- Feedforward Gains ---
    const double intakeDeployS = 0.25;    // Add 0.25 V output to overcome static friction
    const double intakeDeployV = 0.12;    // A velocity target of 1 rps results in 0.12 V output
    const double intakeDeployG = 0.1;     // An acceleration of 1 rps/s requires 0.01 V output

    // --- Motion Magic Profile Parameters ---
    constexpr auto intakeDeployCruiseVelocity = 5_tps;          // 5 (mechanism) rotations per second cruise
    constexpr auto intakeDeployAcceleration = 10_tr_per_s_sq;   // Take approximately 0.5 seconds to reach max vel
    constexpr auto intakeDeployJerk = 100_tr_per_s_cu;          // Take approximately 0.1 seconds to reach max accel
    
    // --- Position Targets (in mechanism rotations) ---
    constexpr auto intakeDeployedPosition = 5.0_tr;   // Arm down / deployed — TODO: measure on real robot
    constexpr auto intakeRetractedPosition = 0.0_tr;  // Arm up / retracted (home position)
        // --- Current Limits ---
    constexpr auto intakeDeploySupplyCurrentLimit = 40.0_A;  // Amps — protects wiring and breakers
    constexpr auto intakeRollerSupplyCurrentLimit = 40.0_A;  // Amps — roller needs less current

    // --- Motor Inversion ---
    const bool intakeDeployRightInverted = true;
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