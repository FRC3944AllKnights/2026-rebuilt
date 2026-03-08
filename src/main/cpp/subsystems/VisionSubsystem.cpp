#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <cmath>

#include "subsystems/VisionSubsystem.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Constants.h"

subsystems::VisionSubsystem::VisionSubsystem() {
    // Load the 2026 field layout from WPILib's built-in resource
    m_fieldLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::k2026RebuiltWelded);
}

void subsystems::VisionSubsystem::Periodic() {
    // Read raw Limelight data every cycle
    m_hasTarget = LimelightHelpers::getTV(VisionConstants::kLimelightName);
    m_tx = LimelightHelpers::getTX(VisionConstants::kLimelightName);
    m_ty = LimelightHelpers::getTY(VisionConstants::kLimelightName);
    m_ta = LimelightHelpers::getTA(VisionConstants::kLimelightName);
    m_tagID = LimelightHelpers::getFiducialID(VisionConstants::kLimelightName);

    // Compute range using per-tag height from field layout
    if (m_hasTarget && m_tagID >= 0) {
        double tagHeight = GetTagHeightInches(m_tagID);
        double heightToCover = tagHeight - ShooterConstants::forwardCameraHeight;
        double angleToCover = m_ty + ShooterConstants::forwardCameraAngle;
        double distanceToTag = heightToCover / std::tan(degreesToRadians(angleToCover));

        m_range = distanceToTag + ShooterConstants::forwardCameraShooterOffset;
    } else {
        m_range = 0.0;
    }

    // --- MegaTag2 Pose Estimation Pipeline ---
    if (m_drivetrain) {
        // Feed Pigeon yaw to Limelight for MegaTag2
        double yawDegrees = m_drivetrain->GetPigeon2().GetYaw().GetValue().value();
        LimelightHelpers::SetRobotOrientation(
            VisionConstants::kLimelightName,
            yawDegrees, 0.0,  // yaw, yawRate
            0.0, 0.0,         // pitch, pitchRate
            0.0, 0.0          // roll, rollRate
        );

        // Read MegaTag2 pose estimate
        m_poseEstimate = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2(
            VisionConstants::kLimelightName
        );
        m_hasValidPose = LimelightHelpers::validPoseEstimate(m_poseEstimate)
                         && m_poseEstimate.tagCount > 0;

        // Feed valid vision pose into drivetrain Kalman filter
        if (m_hasValidPose) {
            // Scale standard deviations by tag count and distance
            // More tags + closer = lower stdDev = more trust
            double xyStdDev = 0.7;  // base standard deviation in meters
            double rotStdDev = 9999999.0; // MegaTag2 handles rotation via IMU, don't trust vision rotation

            if (m_poseEstimate.tagCount >= 2) {
                xyStdDev = 0.5;
            }
            if (m_poseEstimate.avgTagDist > 4.0) {
                // Reduce trust for distant tags (>4 meters)
                xyStdDev *= (m_poseEstimate.avgTagDist / 4.0);
            }

            m_drivetrain->AddVisionMeasurement(
                m_poseEstimate.pose,
                m_poseEstimate.timestampSeconds,
                {xyStdDev, xyStdDev, rotStdDev}
            );
        }
    }

    // Publish to SmartDashboard
    frc::SmartDashboard::PutBoolean("Vision/HasTarget", m_hasTarget);
    frc::SmartDashboard::PutNumber("Vision/TX", m_tx);
    frc::SmartDashboard::PutNumber("Vision/TY", m_ty);
    frc::SmartDashboard::PutNumber("Vision/TA", m_ta);
    frc::SmartDashboard::PutNumber("Vision/Range", m_range);
    frc::SmartDashboard::PutNumber("Vision/TagID", m_tagID);
    frc::SmartDashboard::PutBoolean("Vision/HasValidPose", m_hasValidPose);
    if (m_hasValidPose) {
        frc::SmartDashboard::PutNumber("Vision/PoseX", m_poseEstimate.pose.X().value());
        frc::SmartDashboard::PutNumber("Vision/PoseY", m_poseEstimate.pose.Y().value());
        frc::SmartDashboard::PutNumber("Vision/PoseRot", m_poseEstimate.pose.Rotation().Degrees().value());
        frc::SmartDashboard::PutNumber("Vision/TagCount", m_poseEstimate.tagCount);
        frc::SmartDashboard::PutNumber("Vision/AvgTagDist", m_poseEstimate.avgTagDist);
    }
}

subsystems::VisionSubsystem::visionTarget subsystems::VisionSubsystem::getVisionTarget() {
    // Check if the detected tag is any hub tag (center or offset, either alliance)
    bool isHubTag = false;
    for (int id : AprilTagConstants::kRedHubTags)  { if (m_tagID == id) { isHubTag = true; break; } }
    if (!isHubTag) { for (int id : AprilTagConstants::kBlueHubTags) { if (m_tagID == id) { isHubTag = true; break; } } }

    if (m_hasTarget && isHubTag) {
        // If hub AprilTag visible, determine visionTarget using cached tx/ty

        // Use per-tag height from field layout
        double tagHeight = GetTagHeightInches(m_tagID);
        double heightToCover = tagHeight - ShooterConstants::forwardCameraHeight;
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
    target = subsystems::VisionSubsystem::getVisionTarget();
}

std::optional<frc::Pose3d> subsystems::VisionSubsystem::GetTagFieldPose(int tagId) const {
    return m_fieldLayout.GetTagPose(tagId);
}

double subsystems::VisionSubsystem::GetTagHeightInches(int tagId) const {
    auto pose = m_fieldLayout.GetTagPose(tagId);
    if (pose.has_value()) {
        // Field layout Z is in meters, convert to inches
        return units::inch_t{pose->Z()}.value();
    }
    // Fallback to hardcoded hub tag height if tag not found in layout
    return ShooterConstants::hubTagHeight;
}

double subsystems::VisionSubsystem::degreesToRadians(double degrees) {
    return 0.0174532925 * degrees; // deg * pi / 180
}
