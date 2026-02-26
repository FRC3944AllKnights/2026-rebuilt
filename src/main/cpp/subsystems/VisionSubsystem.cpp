#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <cmath>

#include "subsystems/VisionSubsystem.h"
#include "LimelightHelpers.h"
#include "Constants.h"

subsystems::VisionSubsystem::VisionSubsystem() {
    // Vision Constructor
}

void subsystems::VisionSubsystem::Periodic() {
    // Read raw Limelight data every cycle
    m_hasTarget = LimelightHelpers::getTV(VisionConstants::kLimelightName);
    m_tx = LimelightHelpers::getTX(VisionConstants::kLimelightName);
    m_ty = LimelightHelpers::getTY(VisionConstants::kLimelightName);
    m_ta = LimelightHelpers::getTA(VisionConstants::kLimelightName);
    m_tagID = LimelightHelpers::getFiducialID(VisionConstants::kLimelightName);

    // Compute range using trig whenever any tag is visible
    if (m_hasTarget) {
        double heightToCover = ShooterConstants::hubTagHeight - ShooterConstants::forwardCameraHeight;
        double angleToCover = m_ty + ShooterConstants::forwardCameraAngle;
        double distanceToTag = heightToCover / std::tan(degreesToRadians(angleToCover));

        // Straight-line distance from camera to tag projected on the floor
        m_range = distanceToTag + ShooterConstants::forwardCameraShooterOffset;
    } else {
        m_range = 0.0;
    }

    // Publish to SmartDashboard
    frc::SmartDashboard::PutBoolean("Vision/HasTarget", m_hasTarget);
    frc::SmartDashboard::PutNumber("Vision/TX", m_tx);
    frc::SmartDashboard::PutNumber("Vision/TY", m_ty);
    frc::SmartDashboard::PutNumber("Vision/TA", m_ta);
    frc::SmartDashboard::PutNumber("Vision/Range", m_range);
    frc::SmartDashboard::PutNumber("Vision/TagID", m_tagID);
}

subsystems::VisionSubsystem::visionTarget subsystems::VisionSubsystem::getVisionTarget() {
    // Returns a visionTarget struct consisting of angle and range
    //      angle: field-relative rotation of robot,
    //          assuming starting position is 0 and facing hub from driver station
    //      range: distance from hub in inches (measured from ball exit location to hub center)

    if (m_hasTarget &&
        (m_tagID == ShooterConstants::aprilTagIDHubRed || m_tagID == ShooterConstants::aprilTagIDHubBlue)) {
        // If hub AprilTag visible, determine visionTarget using cached tx/ty

        // Calculate distance from camera to tag
        double heightToCover = ShooterConstants::hubTagHeight - ShooterConstants::forwardCameraHeight;
        double angleToCover = m_ty + ShooterConstants::forwardCameraAngle;
        double distanceToTag = heightToCover / std::tan(degreesToRadians(angleToCover));

        // Calculate distance from shooter to hub center
        double xOffsetFromHub = distanceToTag * std::cos(degreesToRadians(m_tx));
        double yOffsetFromHub = distanceToTag * std::sin(degreesToRadians(m_tx));
        double xOffsetFromTarget = xOffsetFromHub + ShooterConstants::hubWidthMax / 2.0;

        double overheadAngleTarget = std::atan(xOffsetFromTarget / yOffsetFromHub);
        double range = std::sqrt(xOffsetFromTarget * xOffsetFromTarget + yOffsetFromHub * yOffsetFromHub);
        range += ShooterConstants::forwardCameraShooterOffset;

        return visionTarget{overheadAngleTarget, range};
    }
    else {
        // If hub AprilTag not visible, shoot straight ahead, assuming robot is at base position
        double angle = 0.0;
        double range = ShooterConstants::baseRange - ShooterConstants::shooterOffsetFromRearBumper;
        return visionTarget{angle, range};
    }
}

void subsystems::VisionSubsystem::setVisionTarget(visionTarget &target) {
    // Sets given &target based on current Limelight data
    target = subsystems::VisionSubsystem::getVisionTarget();
}

double subsystems::VisionSubsystem::degreesToRadians(double degrees) {
    return 0.0174532925 * degrees; // deg * pi / 180
}
