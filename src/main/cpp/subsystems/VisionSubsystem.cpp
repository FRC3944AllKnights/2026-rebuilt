#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/RobotController.h>

#include "subsystems/VisionSubsystem.h"
#include "LimelightHelpers.h"
#include "Constants.h"

subsystems::VisionSubsystem::VisionSubsystem() {
    // Vision Constructor

    // Stub
    // TODO: Implement
}

subsystems::VisionSubsystem::visionTarget subsystems::VisionSubsystem::getVisionTarget() {
    // Returns a visionTarget struct consisting of angle and range
    //      angle: field-relative rotation of robot,
    //          assuming starting position is 0 and facing hub from driver station
    //      range: distance from hub in inches (measured from ball exit location to hub center)

    int ID = LimelightHelpers::getFiducialID("limelight-forward");

    if (ID == ShooterConstants::aprilTagIDHubRed || ID == ShooterConstants::aprilTagIDHubBlue) {
        // If AprilTag visible, determine visionTarget

        // Pull from Limelight

        double verticalAngleToTag = LimelightHelpers::getTY("limelight-forward");
        double overheadAngleToTag = LimelightHelpers::getTX("limelight-forward");

        // Calculate distance from camera to tag

        double heightToCover = ShooterConstants::hubTagHeight - ShooterConstants::forwardCameraHeight;
        double angleToCover = verticalAngleToTag + ShooterConstants::forwardCameraAngle;
        double distanceToTag = heightToCover / std::tan(degreesToRadians(angleToCover));

        // Calculate distance from shooter to hub center

        double xOffsetFromHub = distanceToTag * std::cos(degreesToRadians(overheadAngleToTag));
        double yOffsetFromHub = distanceToTag * std::sin(degreesToRadians(overheadAngleToTag));
        double xOffsetFromTarget = xOffsetFromHub + ShooterConstants::hubWidthMax / 2.0;

        double overheadAngleTarget = std::atan(xOffsetFromTarget / yOffsetFromHub);
        double range = std::sqrt(xOffsetFromTarget*xOffsetFromTarget + yOffsetFromHub*yOffsetFromHub);
        range += ShooterConstants::forwardCameraShooterOffset;

        // Package and return

        return subsystems::VisionSubsystem::visionTarget(overheadAngleTarget, range);
    }
    else {
        // If AprilTag not visible, shoot straight ahead, assuming robot is at base position

        double angle = 0.0;
        double range = ShooterConstants::baseRange - ShooterConstants::shooterOffsetFromRearBumper;
        return subsystems::VisionSubsystem::visionTarget(angle, range);
    }
}

void subsystems::VisionSubsystem::setVisionTarget(visionTarget &target) {
    // Sets given &target based on current Limelight data
    target = subsystems::VisionSubsystem::getVisionTarget();
}

double subsystems::VisionSubsystem::degreesToRadians(double degrees) {
    return 0.0174532925 * degrees; // deg * pi / 180
}
