#include "subsystems/IntakeSubsystem.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <frc2/command/Commands.h>
#include <iostream>

using namespace ctre::phoenix6;

subsystems::IntakeSubsystem::IntakeSubsystem() {
    // === Deploy Motor Configuration ===
    // Create a configuration object
    configs::TalonFXSConfiguration deployLeftConfig{};
    deployLeftConfig.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

    // --- Motor Type: NEO_JST --- 
    configs::CommutationConfigs &commutation = deployLeftConfig.Commutation;
    commutation.MotorArrangement = ctre::phoenix6::signals::MotorArrangementValue::NEO550_JST;

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
    //softLimits.ForwardSoftLimitEnable = true;
    //softLimits.ForwardSoftLimitThreshold = IntakeConstants::intakeDeployedPosition;
    //softLimits.ReverseSoftLimitEnable = true;
    //softLimits.ReverseSoftLimitThreshold = IntakeConstants::intakeRetractedPosition;

    // --- Motor Outputs: brake on boot ---
    configs::MotorOutputConfigs &motorConfigs = deployLeftConfig.MotorOutput;

    motorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    // Apply configuration to the left deploy motor (leader)
    m_intakeDeployLeftMotor.GetConfigurator().Apply(deployLeftConfig);

    configs::TalonFXSConfiguration deployRightConfig = deployLeftConfig;
    deployRightConfig.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;

    // --- Motor Type: NEO_JST --- 

    configs::CommutationConfigs &rightCommutation = deployRightConfig.Commutation;
    rightCommutation.MotorArrangement = ctre::phoenix6::signals::MotorArrangementValue::NEO550_JST;

    // --- Current Limits: protect motors and wiring ---

    configs::CurrentLimitsConfigs &rightDeployLimits = deployRightConfig.CurrentLimits;

    rightDeployLimits.SupplyCurrentLimitEnable = true;
    rightDeployLimits.SupplyCurrentLimit = IntakeConstants::intakeDeploySupplyCurrentLimit;

    // --- Motor Outputs: brake on boot ---
    configs::MotorOutputConfigs &rightMotorConfigs = deployRightConfig.MotorOutput;
    
    rightMotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        
    m_intakeDeployRightMotor.GetConfigurator().Apply(deployRightConfig);

    // Set the right motor to follow the left motor's output
    // The bool parameter inverts the follower's direction (true = oppose leader)
    //m_intakeDeployRightMotor.SetControl(
    //    controls::Follower{m_intakeDeployLeftMotor.GetDeviceID(), IntakeConstants::intakeDeployRightInverted}
    //);

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

    auto targetPosition = up
        ? IntakeConstants::intakeRetractedPosition
        : IntakeConstants::intakeDeployedPosition;

    m_targetPosition = targetPosition;
    m_intakeDeployLeftMotor.SetControl(
        controls::MotionMagicVoltage{targetPosition}
    );
    m_intakeDeployRightMotor.SetControl(
        controls::MotionMagicVoltage{targetPosition}
    );
}

void subsystems::IntakeSubsystem::SetDeployTarget(units::turn_t position) {
    m_targetPosition = position;
    m_intakeDeployLeftMotor.SetControl(
        controls::MotionMagicVoltage{position}
    );
    m_intakeDeployRightMotor.SetControl(
        controls::MotionMagicVoltage{position}
    );
}

void subsystems::IntakeSubsystem::HoldDeployPosition() {
    m_intakeDeployLeftMotor.SetControl(
        controls::MotionMagicVoltage{m_targetPosition}
    );
    m_intakeDeployRightMotor.SetControl(
        controls::MotionMagicVoltage{m_targetPosition}
    );
}

bool subsystems::IntakeSubsystem::IsAtPosition() {
    auto currentPosition = m_intakeDeployLeftMotor.GetPosition().GetValue();
    return units::math::abs(currentPosition - m_targetPosition) < IntakeConstants::intakePositionTolerance;
}

units::turn_t subsystems::IntakeSubsystem::GetCurrentPosition() {
    return m_intakeDeployLeftMotor.GetPosition().GetValue();
}

units::turn_t subsystems::IntakeSubsystem::GetTargetPosition() {
    return m_targetPosition;
}

void subsystems::IntakeSubsystem::JogPosition(units::turn_t step) {
    m_targetPosition += step;
    m_intakeDeployLeftMotor.SetControl(
        controls::MotionMagicVoltage{m_targetPosition}
    );
    m_intakeDeployRightMotor.SetControl(
        controls::MotionMagicVoltage{m_targetPosition}
    );
}

void subsystems::IntakeSubsystem::PublishTelemetry() {
    frc::SmartDashboard::PutNumber("Intake/Current Position (tr)",
        m_intakeDeployLeftMotor.GetPosition().GetValue().value());
    frc::SmartDashboard::PutNumber("Intake/Target Position (tr)",
        m_targetPosition.value());
}

frc2::CommandPtr subsystems::IntakeSubsystem::PrimeIntakeCommand() {
    return frc2::cmd::Sequence(
        frc2::cmd::RunOnce([this] { SetDeployTarget(IntakeConstants::intakeRetractedPosition); }, {this}),
        frc2::cmd::WaitUntil([this] { return IsAtPosition(); }).WithTimeout(2_s),
        frc2::cmd::RunOnce([this] { SetDeployTarget(IntakeConstants::intakeStartPosition); }, {this}),
        frc2::cmd::WaitUntil([this] { return IsAtPosition(); }).WithTimeout(2_s)
    );
}

