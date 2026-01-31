#pragma once


#include "ctre/phoenix6/SignalLogger.hpp"

#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/Notifier.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>


using namespace ctre::phoenix6;

namespace subsystems {
    class ClimberSubsystem : public frc2::SubsystemBase {
    public:
        ClimberSubsystem();
        void SetClimberPosition(double speed);
    private:
        
    };
} // namespace subsystems