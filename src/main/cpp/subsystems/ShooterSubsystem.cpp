#include "subsystems/ShooterSubsystem.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include "LimelightHelpers.h"

#include <cmath>


subsystems::ShooterSubsystem::ShooterSubsystem() {
    // Shooter Constructor

    // Stub
    // TODO: Implement
}

void subsystems::ShooterSubsystem::SpinUpShooter(double speed) {
    // Set shooter motors to speed
    // speed: speed in rotations per minute (RPM)

    // Stub
    // TODO: Implement
}

void subsystems::ShooterSubsystem::SetIndexerSpeed(double speed) {
    // Set indexer motor speed
    // speed: speed from 0 to 1.0

    // Stub
    // TODO: Implement
}

double subsystems::ShooterSubsystem::getTargetShooterRPM(double rangeIn) {
    // Returns the desired RPM based on vision data
    // range: distance from hub in inches (measured from ball exit location to hub center)
    // if range < minimumRange: Calculate directly from vision data (including for negative inputs)
    // if AprilTag not visible: Calculate from base position

    double range;
    if (rangeIn < ShooterConstants::minimumRange) {
        range = ShooterConstants::minimumRange;
    }
    else {
        range = rangeIn;
    }

    double b;
    if (ShooterConstants::exitAngle == 45.0) {
        b = 1.0;
    }
    else if (ShooterConstants::exitAngle == 60.0) {
        b = 1.73205;
    }
    else {
        b = std::tan(ShooterConstants::exitAngle * 3.14159 / 180.0);
    }
    double a = (ShooterConstants::hubHeight - ShooterConstants::shooterHeight - b * range) / (range * range);
    double c = 2.0 * ShooterConstants::shooterHeight;
    double determinant = b * b - 4.0 * a * c;
    double range_floor_to_floor = -1.0 * b - std::sqrt(determinant) / (2.0 * a);
    double exit_speed = std::sqrt(range_floor_to_floor * ShooterConstants::g / std::sin(b));
    double rpm_nominal = exit_speed / ShooterConstants::shooterCircumference;
    return rpm_nominal * ShooterConstants::shooterBallSpeedTransferPercent;
}

double subsystems::ShooterSubsystem::degreesToRadians(double degrees) {
    return 0.0174532925 * degrees; // deg * pi / 180
}
