#pragma once

#include "ctre/phoenix6/SignalLogger.hpp"
#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/controls/MotionMagicVoltage.hpp"
#include "ctre/phoenix6/controls/DutyCycleOut.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"

#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/Notifier.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include "ctre/phoenix6/TalonFXS.hpp"

using namespace ctre::phoenix6::hardware;

namespace subsystems {
    class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem();
        void RunIntake(double speed);
        void SetIntakePosition(bool up);
        void DisengageLatches();
        void HoldDeployPosition();

    private:

        // Members - Motors
        
        TalonFXS m_intakeDeployLeftMotor{CANConstants::kIntakeDeployLeftMotorId, CANConstants::kCANBusName};
        TalonFXS m_intakeDeployRightMotor{CANConstants::kIntakeDeployRightMotorId, CANConstants::kCANBusName};
        TalonFXS m_intakeRollerMotor{CANConstants::kIntakeRollerMotorId, CANConstants::kCANBusName};

        units::turn_t m_targetPosition{IntakeConstants::intakeRetractedPosition};

        bool m_hasDisengagedLatches = false;
        
};
} // namespace subsystems