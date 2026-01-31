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

//#include "Constants.h" // Commented out - we do not know if we need a Constants.h in CTRE

//using namespace IntakeConstants; // Commented out - we do not know if we need a Constants.h in CTRE
namespace subsystems {
class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();
    void RunIntake(bool forward);
private:
    
};
} // namespace subsystems