#pragma once


#include "ctre/phoenix6/SignalLogger.hpp"

#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/Notifier.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFXS.hpp"

#include "Constants.h"

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::configs;

using namespace ctre::phoenix6::controls;

namespace subsystems {
    class ShooterSubsystem : public frc2::SubsystemBase {
    public:

        // Methods

        ShooterSubsystem();
        void SpinUpShooter(double speed);
        void SetIndexerSpeed(double speed);
        double getTargetShooterRPM(double rangeIn);

    private:

        // Methods

        double degreesToRadians(double degrees);

        // Members - Motors
        
        TalonFXS m_shooterLeftMotor{CANConstants::kShooterLeftMotorId, CANConstants::kCANBusName};
        TalonFXS m_shooterRightMotor{CANConstants::kShooterRightMotorId, CANConstants::kCANBusName};
        TalonFXS m_indexerMotor{CANConstants::kIndexerMotorId, CANConstants::kCANBusName};

        // Members - Configs

        TalonFXSConfiguration m_shooterLeftMotorConfig;
        TalonFXSConfiguration m_shooterRightMotorConfig;
        TalonFXSConfiguration m_indexerMotorConfig;
    
    };
} // namespace subsystems
