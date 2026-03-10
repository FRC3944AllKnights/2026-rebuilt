#pragma once


#include "ctre/phoenix6/SignalLogger.hpp"

#include <frc/Timer.h>
#include <frc/DriverStation.h>
#include <frc/Notifier.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFXS.hpp"
#include <iostream>

#include "Constants.h"
#include "subsystems/LEDStrip.h"
#include "subsystems/VisionSubsystem.h"

using namespace ctre::phoenix6::hardware;
using namespace ctre::phoenix6::configs;

using namespace ctre::phoenix6::controls;

namespace subsystems {
    class ShooterSubsystem : public frc2::SubsystemBase {
    public:

        // Methods

        ShooterSubsystem();
        void Periodic() override;
        void SpinUpShooter(double speed);
        void SetIndexerSpeed(double speed);
        double getTargetShooterRPM(double rangeIn);
        void SetVision(VisionSubsystem* vision) { m_vision = vision; }
        void SetAdjustableRPM(bool enabled) { m_adjustableRPM = enabled; }
        void ToggleAdjustableRPM() { m_adjustableRPM = !m_adjustableRPM; }
        bool IsAdjustableRPMEnabled() const { return m_adjustableRPM; }

    private:

        // Methods

        double degreesToRadians(double degrees);
        void setLEDs(char patternID);

        // Members - Devices
        
        TalonFXS m_shooterLeftMotor{CANConstants::kShooterLeftMotorId, CANConstants::kCANBusName};
        TalonFXS m_shooterRightMotor{CANConstants::kShooterRightMotorId, CANConstants::kCANBusName};
        TalonFXS m_indexerMotor{CANConstants::kIndexerMotorId, CANConstants::kCANBusName};
        LEDStrip m_LEDStrip{ShooterConstants::LEDPort, ShooterConstants::kLength};

        // Members - Configs

        TalonFXSConfiguration m_shooterLeftMotorConfig;
        TalonFXSConfiguration m_shooterRightMotorConfig;
        TalonFXSConfiguration m_indexerMotorConfig;

        VisionSubsystem* m_vision = nullptr;
        bool m_adjustableRPM = false;

        /*
        frc::AddressableLED m_led{1};
        std::array<frc::AddressableLED::LEDData, ShooterConstants::kLength> m_ledBuffer;
        frc::LEDPattern patternYellow = frc::LEDPattern::Solid(frc::Color::kYellow);
        frc::LEDPattern patternBlue = frc::LEDPattern::Solid(frc::Color::kBlue);
        frc::LEDPattern patternWhite = frc::LEDPattern::Solid(frc::Color::kWhite);
        */
    };
} // namespace subsystems
