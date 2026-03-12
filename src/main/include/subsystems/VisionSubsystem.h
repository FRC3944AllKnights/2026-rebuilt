#pragma once

#include <frc/DriverStation.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc2/command/SubsystemBase.h>
#include <optional>
#include <units/length.h>

#include "LimelightHelpers.h"

namespace subsystems {

    // Forward declaration
    class CommandSwerveDrivetrain;

    class VisionSubsystem : public frc2::SubsystemBase {
    public:
        VisionSubsystem();

        void Periodic() override;

        struct visionTarget {
            double overheadAngle; // radians - robot spin
            double range; // inches - distance from shooter to hub center
        };

        visionTarget getVisionTarget();
        void setVisionTarget(visionTarget &target);

        // Field layout accessors
        std::optional<frc::Pose3d> GetTagFieldPose(int tagId) const;
        double GetTagHeightInches(int tagId) const;
        const frc::AprilTagFieldLayout& GetFieldLayout() const { return m_fieldLayout; }

        // MegaTag2 pose estimation
        LimelightHelpers::PoseEstimate GetLatestPoseEstimate() const { return m_poseEstimate; }
        bool HasValidPoseEstimate() const { return m_hasValidPose; }

        // Set drivetrain reference for IMU heading and vision fusion
        void SetDrivetrain(CommandSwerveDrivetrain* drivetrain) { m_drivetrain = drivetrain; }

        bool HasValidShooterTarget() const;
        double GetTX() const;

    private:
        double degreesToRadians(double degrees);

        // Field layout loaded from WPILib built-in resource
        frc::AprilTagFieldLayout m_fieldLayout;

        // Drivetrain reference for Pigeon yaw + vision measurement fusion
        CommandSwerveDrivetrain* m_drivetrain = nullptr;

        // MegaTag2 pose estimation
        LimelightHelpers::PoseEstimate m_poseEstimate;
        bool m_hasValidPose = false;

        // Cached values updated every cycle in Periodic()
        bool m_hasTarget = false;
        bool m_hasShooterTarget = false;
        double m_tx = 0.0;
        double m_ty = 0.0;
        double m_ta = 0.0;
        double m_range = 0.0;
        int m_tagID = -1;
    };
} // namespace subsystems