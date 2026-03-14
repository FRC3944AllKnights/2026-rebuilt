#include "subsystems/ShooterSubsystem.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include "LimelightHelpers.h"

#include <cmath>


subsystems::ShooterSubsystem::ShooterSubsystem() {
    // Shooter Constructor

    // Left shooter motor
    // Directly controlled motor, requires PID configuration for velocity control

    TalonFXSConfiguration shooterLeftMotorConfig{};
    shooterLeftMotorConfig.Commutation.MotorArrangement = ctre::phoenix6::signals::MotorArrangementValue::NEO_JST;
    shooterLeftMotorConfig.Slot0.WithKP(ShooterConstants::shooterP);
    shooterLeftMotorConfig.Slot0.WithKI(ShooterConstants::shooterI);
    shooterLeftMotorConfig.Slot0.WithKD(ShooterConstants::shooterD);
    shooterLeftMotorConfig.Slot0.WithKS(ShooterConstants::shooterS);
    shooterLeftMotorConfig.Slot0.WithKV(ShooterConstants::shooterV); // TODO: Measure. Found value online
    shooterLeftMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0_A; // TODO: Determine appropriate current limit
    shooterLeftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_shooterLeftMotor.GetConfigurator().Apply(shooterLeftMotorConfig);

    // Right shooter motor
    // Follower motor, copies output of left motor

    TalonFXSConfiguration shooterRightMotorConfig{};
    shooterRightMotorConfig.Commutation.MotorArrangement = ctre::phoenix6::signals::MotorArrangementValue::NEO_JST;
    shooterRightMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0_A; // TODO: Determine appropriate current limit
    shooterRightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_shooterRightMotor.GetConfigurator().Apply(shooterRightMotorConfig);

    bool followerModeEnabled = true;
    if (followerModeEnabled) {
        bool invertRightMotor = true; // Set to true if right motor needs to be inverted to match left motor's direction
        Follower rightMotorControlMethod(CANConstants::kShooterLeftMotorId, invertRightMotor); 
        m_shooterRightMotor.SetControl(rightMotorControlMethod);
    }

    // Indexer motor

    TalonFXSConfiguration indexerLeftMotorConfig{};
    indexerLeftMotorConfig.Commutation.MotorArrangement = ctre::phoenix6::signals::MotorArrangementValue::NEO_JST;
    indexerLeftMotorConfig.Slot0.WithKP(ShooterConstants::indexerP);
    indexerLeftMotorConfig.Slot0.WithKI(ShooterConstants::indexerI);
    indexerLeftMotorConfig.Slot0.WithKD(ShooterConstants::indexerD);
    indexerLeftMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0_A; // TODO: Determine appropriate current limit
    indexerLeftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_indexerLeftMotor.GetConfigurator().Apply(indexerLeftMotorConfig);

    TalonFXSConfiguration indexerRightMotorConfig{};
    indexerRightMotorConfig.Commutation.MotorArrangement = ctre::phoenix6::signals::MotorArrangementValue::NEO_JST;
    indexerRightMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0_A; // TODO: Determine appropriate current limit
    indexerRightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_indexerRightMotor.GetConfigurator().Apply(indexerRightMotorConfig);   

    Follower rightIndexerMotorControlMethod(CANConstants::kIndexerLeftMotorId, true); 
    m_indexerRightMotor.SetControl(rightIndexerMotorControlMethod);

    // LED Strip
    //m_LEDStrip = LEDStrip(ShooterConstants::LEDPort, ShooterConstants::kLength);
    /*
    m_led.SetLength(ShooterConstants::kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
    */
}

void subsystems::ShooterSubsystem::Periodic() {
    // Passive debug telemetry — publish computed RPM without requiring trigger input
    frc::SmartDashboard::PutBoolean("Shooter/AdjustableRPMEnabled", m_adjustableRPM);

    if (m_vision != nullptr) {
        bool isShootable = m_vision->HasValidShooterTarget();
        if (m_adjustableRPM && isShootable) {
            auto target = m_vision->getVisionTarget();
            double computedRPM = getTargetShooterRPM(target.range);
            frc::SmartDashboard::PutNumber("Shooter/ComputedRPM", computedRPM);
        } else {
            frc::SmartDashboard::PutNumber("Shooter/ComputedRPM", 0.0);
        }
    }
}

void subsystems::ShooterSubsystem::SpinUpShooter(double speed) {
    // Set shooter motors to speed
    // speed: speed from 0 to 1.0

    // TODO: Implement proper RPM determination
    // Currently for prototyping only



    // Determine target RPM

    units::revolutions_per_minute_t baseRPM = 2950.0_rpm; // For prototyping only
    double gearRatio = 1.0; // Torque multiplier
    units::revolutions_per_minute_t targetRPM;

    if (m_adjustableRPM && m_vision != nullptr && speed > 0.01) {
        if (m_vision->HasValidShooterTarget()) {
            auto target = m_vision->getVisionTarget();
            double computedRPM = getTargetShooterRPM(target.range);
            targetRPM = units::revolutions_per_minute_t(computedRPM);
        } else {
            targetRPM = baseRPM / gearRatio;
        }
    } else {
        targetRPM = speed * baseRPM / gearRatio;
    }

    units::turns_per_second_t targetTPS = targetRPM;

    // Apply to motor

    if (speed <= 0.01) {
        m_shooterLeftMotor.SetControl(NeutralOut{});
    } else {
        VelocityVoltage m_request{targetTPS};
        m_shooterLeftMotor.SetControl(m_request.WithVelocity(targetTPS).WithSlot(0));
    }

    // Output to dashboard for testing
    
    frc::SmartDashboard::PutNumber("Shooter Wheel Target RPM", targetRPM.value());
    double actualRPM = m_shooterLeftMotor.GetRotorVelocity().GetValue().value() * 60.0;
    frc::SmartDashboard::PutNumber("Shooter Wheel Actual RPM", actualRPM);

    if (ShooterConstants::debugPrintsEnabled) {
        frc::SmartDashboard::PutNumber("Speed Commanded [0, 1]", speed);

        double leftSupplyVoltage = m_shooterLeftMotor.GetSupplyVoltage().GetValue().value();
        double rightSupplyVoltage = m_shooterRightMotor.GetSupplyVoltage().GetValue().value();
        frc::SmartDashboard::PutNumber("Shooter Left Supply Voltage", leftSupplyVoltage);
        frc::SmartDashboard::PutNumber("Shooter Right Supply Voltage", rightSupplyVoltage);

        double rightMotorRPM = m_shooterRightMotor.GetRotorVelocity().GetValue().value() * 60.0;
        double percentDifferenceShooterMotors = 100.0 * (actualRPM - rightMotorRPM) / ((actualRPM + rightMotorRPM) / 2.0);
        frc::SmartDashboard::PutNumber("Shooter Motors RPM Percent Difference", percentDifferenceShooterMotors);

        double leftMotorPosition = m_shooterLeftMotor.GetRotorPosition().GetValue().value();
        double rightMotorPosition = m_shooterRightMotor.GetRotorPosition().GetValue().value();
        double percentDifferenceShooterMotorsTheta = 100.0 * (leftMotorPosition - rightMotorPosition) / ((leftMotorPosition + rightMotorPosition) / 2.0);
        frc::SmartDashboard::PutNumber("Shooter Motors theta Percent Difference", percentDifferenceShooterMotorsTheta);
    }

    // Set LED if shooter is commanded
    /*
    if (speed > 0.0) {
        m_LEDStrip.setLEDs('Y'); // Set to yellow when shooter is commanded
    }
     else {
        m_LEDStrip.setLEDs('B'); // Set to blue when shooter is not commanded
    }
        */
}

void subsystems::ShooterSubsystem::SetIndexerSpeed(double speed) {
    // Set indexer motor speed
    // speed: speed from 0 to 1.0

    if (speed <= 0.01) {
        m_indexerLeftMotor.SetControl(NeutralOut{});
    } else {
        m_indexerLeftMotor.SetControl(DutyCycleOut{speed});
    }

    if (ShooterConstants::debugPrintsEnabled) {
        frc::SmartDashboard::PutNumber("Indexer Commanded Speed", speed);
        double actualRPM = m_indexerLeftMotor.GetRotorVelocity().GetValue().value() * 60.0;
        frc::SmartDashboard::PutNumber("Indexer Actual RPM", actualRPM);
    }
}

double subsystems::ShooterSubsystem::getTargetShooterRPM(double rangeIn) {
    // Returns the desired RPM based on vision data
    // range: distance from hub in inches (measured from ball exit location to hub center)
    // if range < minimumRange: Calculate directly from vision data (including for negative inputs)
    // if AprilTag not visible: Calculate from base position

    double range;
    if (rangeIn < ShooterConstants::minimumRange) {
        range = ShooterConstants::minimumRange;
    }
    else {
        range = rangeIn;
    }

    double b;
    if (ShooterConstants::exitAngle == 45.0) {
        b = 1.0;
    }
    else if (ShooterConstants::exitAngle == 60.0) {
        b = 1.73205;
    }
    else {
        b = std::tan(ShooterConstants::exitAngle * 3.14159 / 180.0);
    }
    double a = (ShooterConstants::hubHeight - ShooterConstants::shooterHeight - b * range) / (range * range);

    // Extract exit velocity directly from trajectory coefficient a.
    // a encodes -g / (2 * v² * cos²θ), so solve for v:
    double cosTheta = std::cos(ShooterConstants::exitAngle * 3.14159 / 180.0);
    double exit_speed = std::sqrt(-ShooterConstants::g / (2.0 * a * cosTheta * cosTheta));
    double rpm_nominal = (exit_speed / ShooterConstants::shooterCircumference) * 60.0;

    if (ShooterConstants::debugPrintsEnabled) {
        frc::SmartDashboard::PutNumber("Shooter/Debug/InputRange", rangeIn);
        frc::SmartDashboard::PutNumber("Shooter/Debug/ClampedRange", range);
        frc::SmartDashboard::PutNumber("Shooter/Debug/CoeffA", a);
        frc::SmartDashboard::PutNumber("Shooter/Debug/ExitSpeedInPerSec", exit_speed);
        frc::SmartDashboard::PutNumber("Shooter/Debug/RPMNominal", rpm_nominal);
    }

    return rpm_nominal / ShooterConstants::shooterBallSpeedTransferPercent;
}

double subsystems::ShooterSubsystem::degreesToRadians(double degrees) {
    return 0.0174532925 * degrees; // deg * pi / 180
}

/*
void subsystems::ShooterSubsystem::setLEDs(char patternID) {
    // Set LEDs to a specific pattern based on input ID
    // patternID: character representing the desired pattern (e.g., 'Y' for yellow, 'B' for blue)
    frc::LEDPattern& pattern = patternWhite; // Default pattern
    if (patternID == 'Y') {
        pattern = patternYellow;
    }
    else if (patternID == 'B') {
        pattern = patternBlue;
    }
    pattern.ApplyTo(m_ledBuffer);
    m_led.SetData(m_ledBuffer);
}
*/