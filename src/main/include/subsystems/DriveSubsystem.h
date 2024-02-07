// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/acceleration.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/acceleration.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/Encoder.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/geometry/Pose2d.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/RobotController.h>
#include <AHRS.h>

using namespace units;

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  void JoystickDrive(double xSpeed, double ySpeed, bool turnInPlace);
  void Drive(frc::DifferentialDriveWheelSpeeds wheelSpeeds);
  void Drive(units::meters_per_second_t xSpeed, units::radians_per_second_t rot);
  void Drive(frc::ChassisSpeeds chassisSpeed);
  void StopMotors();
  frc::ChassisSpeeds GetSpeed();

  double GetLeftEncoder();
  double GetRightEncoder();
  void ResetOdometry(frc::Pose2d pose);
  void UpdateOdometry();
  frc::Pose2d GetPose();

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

    const int lmotor_pwm_channel_1 = 9;
    const int lmotor_pwm_channel_2 = 8;
    const int rmotor_pwm_channel_1 = 0;
    const int rmotor_pwm_channel_2 = 1;
    const int conv_pwm_channel = 2;

    frc::PWMSparkMax m_leftMotor1{lmotor_pwm_channel_1};
    frc::PWMSparkMax m_rightMotor1{rmotor_pwm_channel_1};
    frc::PWMSparkMax m_leftMotor2{lmotor_pwm_channel_2};
    frc::PWMSparkMax m_rightMotor2{rmotor_pwm_channel_2};
    frc::PWMSparkMax m_conveyorMotor{conv_pwm_channel};
    frc::SlewRateLimiter<units::dimensionless::scalar> filter{2/1_s};

    frc::Encoder m_leftEncoder{1, 2};
    frc::Encoder m_rightEncoder{7, 8};

    AHRS m_gyro{frc::SPI::Port::kMXP};

    frc::DifferentialDrive m_robotDrive{m_leftMotor1, m_rightMotor1};
    frc::DifferentialDriveKinematics m_kinematics{28_in};
    frc::DifferentialDriveOdometry m_odometry{{}, units::meter_t{m_leftEncoder.GetDistance()}, units::meter_t{m_rightEncoder.GetDistance()}};
    frc::PIDController m_leftPIDController{1.0, 0.0, 0.0};
    frc::PIDController m_rightPIDController{1.0, 0.0, 0.0};
    frc::SimpleMotorFeedforward<units::meters> m_feedforward{0.2_V, 3_V / 1_mps};


    frc2::sysid::SysIdRoutine m_sysIdRoutine{
        frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt,
                            std::nullopt},
        frc2::sysid::Mechanism{
            [this](units::volt_t driveVoltage) {
              m_leftMotor1.SetVoltage(driveVoltage);
              m_rightMotor1.SetVoltage(driveVoltage);
              m_leftMotor2.SetVoltage(driveVoltage);
              m_rightMotor2.SetVoltage(driveVoltage);
            },
            [this](frc::sysid::SysIdRoutineLog* log) {
              log->Motor("drive-left-1")
                  .voltage(m_leftMotor1.Get() *
                            frc::RobotController::GetBatteryVoltage())
                  .position(units::meter_t{m_leftEncoder.GetDistance()})
                  .velocity(units::meters_per_second_t{m_leftEncoder.GetRate()});
              log->Motor("drive-right-1")
                  .voltage(m_rightMotor1.Get() *
                            frc::RobotController::GetBatteryVoltage())
                  .position(units::meter_t{m_rightEncoder.GetDistance()})
                  .velocity(units::meters_per_second_t{m_rightEncoder.GetRate()});
              log->Motor("drive-left-2")
                  .voltage(m_leftMotor2.Get() *
                            frc::RobotController::GetBatteryVoltage())
                  .position(units::meter_t{m_leftEncoder.GetDistance()})
                  .velocity(units::meters_per_second_t{m_leftEncoder.GetRate()});
              log->Motor("drive-right-2")
                  .voltage(m_rightMotor2.Get() *
                            frc::RobotController::GetBatteryVoltage())
                  .position(units::meter_t{m_rightEncoder.GetDistance()})
                  .velocity(units::meters_per_second_t{m_rightEncoder.GetRate()});
            },
            this}};
};