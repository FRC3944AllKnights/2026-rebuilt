#pragma once

#include <frc/DriverStation.h>
#include <frc2/command/SubsystemBase.h>

namespace subsystems {
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

    private:
        double degreesToRadians(double degrees);

        // Cached values updated every cycle in Periodic()
        bool m_hasTarget = false;
        double m_tx = 0.0;
        double m_ty = 0.0;
        double m_ta = 0.0;
        double m_range = 0.0;
        int m_tagID = -1;
    };
} // namespace subsystems