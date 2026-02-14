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
    shooterLeftMotorConfig.Slot0.WithKP(ShooterConstants::shooterP);
    shooterLeftMotorConfig.Slot0.WithKI(ShooterConstants::shooterI);
    shooterLeftMotorConfig.Slot0.WithKD(ShooterConstants::shooterD);
    shooterLeftMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0_A; // TODO: Determine appropriate current limit
    shooterLeftMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_shooterLeftMotor.GetConfigurator().Apply(m_shooterLeftMotorConfig);

    // Right shooter motor
    // Follower motor, copies output of left motor

    TalonFXSConfiguration shooterRightMotorConfig{};
    shooterRightMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0_A; // TODO: Determine appropriate current limit
    shooterRightMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_shooterRightMotor.GetConfigurator().Apply(shooterRightMotorConfig);
    bool invertRightMotor = true; // Set to true if right motor needs to be inverted to match left motor's direction
    Follower rightMotorControlMethod(CANConstants::kShooterRightMotorId, invertRightMotor); 
    m_shooterRightMotor.SetControl(rightMotorControlMethod);

    // Indexer motor

    TalonFXSConfiguration indexerMotorConfig{};
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

    units::revolutions_per_minute_t baseRPM = 2000.0_rpm; // For prototyping only
    units::revolutions_per_minute_t targetRPM = speed * baseRPM;
    units::turns_per_second_t targetTPS = targetRPM / 60.0; // Convert RPM to TPS (turns per second)

    // Apply to motor

    VelocityVoltage m_request = VelocityVoltage{0_tps}.WithSlot(0);
    m_shooterLeftMotor.SetControl(m_request.WithVelocity(targetTPS));

    // Output to dashboard for testing
    
    frc::SmartDashboard::PutNumber("Shooter Target RPM", targetRPM.value());
    double actualRPM = m_shooterLeftMotor.GetVelocity().GetValue().value() * 60.0;
    frc::SmartDashboard::PutNumber("Shooter Actual RPM", actualRPM);
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
