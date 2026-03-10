#include "subsystems/IntakeSubsystem.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <iostream>

using namespace ctre::phoenix6;

subsystems::IntakeSubsystem::IntakeSubsystem() {
    // === Deploy Motor Configuration ===
    // Create a configuration object
    configs::TalonFXSConfiguration deployLeftConfig{};

    // --- Motor Type: NEO_JST --- 
    configs::CommutationConfigs &commutation = deployLeftConfig.Commutation;
    commutation.MotorArrangement = ctre::phoenix6::signals::MotorArrangementValue::NEO_JST;

    // --- PID Slot 0: closed-loop position control gains ---
    configs::Slot0Configs &slot0 = deployLeftConfig.Slot0;

    slot0.kP = IntakeConstants::intakeDeployP;
    slot0.kI = IntakeConstants::intakeDeployI;
    slot0.kD = IntakeConstants::intakeDeployD;
    slot0.kS = IntakeConstants::intakeDeployS;
    slot0.kV = IntakeConstants::intakeDeployV;
    slot0.kG = IntakeConstants::intakeDeployG;

    // --- Motion Magic: smooth trapezoidal/S-curve motion profile ---
    configs::MotionMagicConfigs &motionMagic = deployLeftConfig.MotionMagic;

    motionMagic.MotionMagicCruiseVelocity = IntakeConstants::intakeDeployCruiseVelocity;
    motionMagic.MotionMagicAcceleration = IntakeConstants::intakeDeployAcceleration;
    motionMagic.MotionMagicJerk = IntakeConstants::intakeDeployJerk; 

    // --- Current Limits: protect motors and wiring ---
    configs::CurrentLimitsConfigs &deployLimits = deployLeftConfig.CurrentLimits;

    deployLimits.SupplyCurrentLimitEnable = true;
    deployLimits.SupplyCurrentLimit = IntakeConstants::intakeDeploySupplyCurrentLimit;

    // --- Feedback: account for gear reduction between motor and arm ---
    configs::ExternalFeedbackConfigs &externalFeedback = deployLeftConfig.ExternalFeedback;

    externalFeedback.SensorToMechanismRatio = IntakeConstants::intakeDeployGearRatio;

    // --- Software Limits: prevent mechanism from trying to move beyond physical limits ---
    configs::SoftwareLimitSwitchConfigs &softLimits = deployLeftConfig.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ForwardSoftLimitThreshold = IntakeConstants::intakeDeployedPosition;
    softLimits.ReverseSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = IntakeConstants::intakeRetractedPosition;

    // Apply configuration to the left deploy motor (leader)
    m_intakeDeployLeftMotor.GetConfigurator().Apply(deployLeftConfig);

    configs::TalonFXSConfiguration deployRightConfig{};

    // --- Motor Type: NEO_JST --- 

    configs::CommutationConfigs &commutation = deployRightConfig.Commutation;
    commutation.MotorArrangement = ctre::phoenix6::signals::MotorArrangementValue::NEO_JST;

    // --- Current Limits: protect motors and wiring ---

    configs::CurrentLimitsConfigs &deployLimits = deployRightConfig.CurrentLimits;

    deployLimits.SupplyCurrentLimitEnable = true;
    deployLimits.SupplyCurrentLimit = IntakeConstants::intakeDeploySupplyCurrentLimit;
        
    m_intakeDeployRightMotor.GetConfigurator().Apply(deployRightConfig);

    // Set the right motor to follow the left motor's output
    // The bool parameter inverts the follower's direction (true = oppose leader)
    m_intakeDeployRightMotor.SetControl(
        controls::Follower{m_intakeDeployLeftMotor.GetDeviceID(), IntakeConstants::intakeDeployRightInverted}
    );

    // === Roller Motor Configuration ===
    configs::TalonFXSConfiguration rollerConfig{};
    configs::CurrentLimitsConfigs &rollerLimits = rollerConfig.CurrentLimits;
    rollerConfig.Commutation.MotorArrangement = ctre::phoenix6::signals::MotorArrangementValue::NEO_JST;
    rollerLimits.SupplyCurrentLimitEnable = true;
    rollerLimits.SupplyCurrentLimit = IntakeConstants::intakeRollerSupplyCurrentLimit;
    m_intakeRollerMotor.GetConfigurator().Apply(rollerConfig);

    // Zero the deploy encoder so "0" = current starting position (retracted)
    m_intakeDeployLeftMotor.SetPosition(0_tr);
    m_intakeDeployRightMotor.SetPosition(0_tr);
}

void subsystems::IntakeSubsystem::RunIntake(double speed) {
    // Power intake motors
    // speed: [-1.0, 1.0] for reverse to forward
    // DutyCycleOut sends a percentage of available voltage — no feedback loop
    m_intakeRollerMotor.SetControl(controls::DutyCycleOut{speed});
}

void subsystems::IntakeSubsystem::SetIntakePosition(bool up) {
    // Set intake position using Motion Magic — smooth, profiled position control
    // up: true = retracted (stowed), false = deployed (down to collect game pieces)
    // MotionMagicVoltage tells the motor controller to go to an exact position
    // following the trapezoidal/S-curve profile configured in the constructor
    std::cout << "In set intake position" << std::endl;
    auto targetPosition = up
        ? IntakeConstants::intakeRetractedPosition
        : IntakeConstants::intakeDeployedPosition;
    m_intakeDeployLeftMotor.SetControl(
        controls::MotionMagicVoltage{targetPosition}
    );
    // Right motor follows automatically via the Follower set in constructor
}