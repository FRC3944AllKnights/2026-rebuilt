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
    shooterLeftMotorConfig.Slot0.WithKV(0.12); // TODO: Measure. Found value online
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
    bool invertRightMotor = true; // Set to true if right motor needs to be inverted to match left motor's direction
    Follower rightMotorControlMethod(CANConstants::kShooterLeftMotorId, invertRightMotor);
    m_shooterRightMotor.SetControl(rightMotorControlMethod);

    // Sets follower mode
    // TODO: Reset to true after code validated on left motor
    bool followerModeEnabled = false;
    if (followerModeEnabled) {
        Follower rightMotorControlMethod(CANConstants::kShooterLeftMotorId, invertRightMotor); 
        m_shooterRightMotor.SetControl(rightMotorControlMethod);
    }

    // Indexer motor

    TalonFXSConfiguration indexerMotorConfig{};
    indexerMotorConfig.Commutation.MotorArrangement = ctre::phoenix6::signals::MotorArrangementValue::NEO_JST;
    indexerMotorConfig.Slot0.WithKP(ShooterConstants::indexerP);
    indexerMotorConfig.Slot0.WithKI(ShooterConstants::indexerI);
    indexerMotorConfig.Slot0.WithKD(ShooterConstants::indexerD);
    indexerMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0_A; // TODO: Determine appropriate current limit
    indexerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_indexerMotor.GetConfigurator().Apply(indexerMotorConfig);

}

void subsystems::ShooterSubsystem::SpinUpShooter(double speed) {
    // Set shooter motors to speed
    // speed: speed from 0 to 1.0

    // TODO: Implement proper RPM determination
    // Currently for prototyping only

    // Determine target RPM

    std::cout << "In spin up shooter method" << std::endl;

    units::revolutions_per_minute_t baseRPM = 2000.0_rpm; // For prototyping only
    double gearRatio = 1.0; // Torque multiplier
    units::revolutions_per_minute_t targetRPM = speed * baseRPM / gearRatio;
    units::turns_per_second_t targetTPS = targetRPM; // Convert RPM to TPS (turns per second)

    // Apply to motor

    if (speed <= 0.01) {
        m_shooterLeftMotor.SetControl(NeutralOut{});
    } else {
        VelocityVoltage m_request{targetTPS};
        m_shooterLeftMotor.SetControl(m_request.WithVelocity(targetTPS).WithSlot(0));
    }

    std::cout << "Sets control" << std::endl;

    // Output to dashboard for testing
    
    frc::SmartDashboard::PutNumber("Shooter Wheel Target RPM", targetRPM.value());
    double actualRPM = m_shooterLeftMotor.GetRotorVelocity().GetValue().value() * 60.0;
    frc::SmartDashboard::PutNumber("Shooter Wheel Actual RPM", actualRPM);
    std::cout << "Shooter Wheel Target RPM: " << targetRPM.value() << std::endl;
    std::cout << "Shooter Wheel Actual RPM: " << actualRPM << std::endl;

    if (ShooterConstants::debugPrintsEnabled) {
        frc::SmartDashboard::PutNumber("Speed Commanded [0, 1]", speed);
        std::cout << "Speed Commanded [0, 1]: " << speed << std::endl;

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
}

void subsystems::ShooterSubsystem::SetIndexerSpeed(double speed) {
    // Set indexer motor speed
    // speed: speed from 0 to 1.0

    // Stub
    // TODO: Implement
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
    double c = 2.0 * ShooterConstants::shooterHeight;
    double determinant = b * b - 4.0 * a * c;
    double range_floor_to_floor = -1.0 * b - std::sqrt(determinant) / (2.0 * a);
    double exit_speed = std::sqrt(range_floor_to_floor * ShooterConstants::g / std::sin(b));
    double rpm_nominal = exit_speed / ShooterConstants::shooterCircumference;
    return rpm_nominal * ShooterConstants::shooterBallSpeedTransferPercent;
}

double subsystems::ShooterSubsystem::degreesToRadians(double degrees) {
    return 0.0174532925 * degrees; // deg * pi / 180
}
