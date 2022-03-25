#pragma once

#ifndef MECANUM_SUBSYSTEM_H
#define MECANUM_SUBSYSTEM_H

#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>

#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/Encoder.h>
#include <frc/AnalogGyro.h>

#include <frc/simulation/PWMSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/AnalogGyroSim.h>

#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>
#include <units/angle.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>

#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

class MecanumSubsystem : public frc2::SubsystemBase {
 public:
  MecanumSubsystem();

  void Periodic() override;
  void SimulationPeriodic() override;

  void Drive(units::meters_per_second_t vx, units::meters_per_second_t vy, units::radians_per_second_t omega);
  void Drive(units::meters_per_second_t left, units::meters_per_second_t right);
  void DriveVoltages(units::volt_t _fl, units::volt_t _fr, units::volt_t _bl, units::volt_t _br);
  void Drive(units::meters_per_second_t vy, units::radians_per_second_t omega);
  void DriveVoltages(units::volt_t left, units::volt_t right);

  void ResetEncoders();

  units::degree_t GetAngle();
  double GetAngleRaw();

  frc::Pose2d GetPose();

  frc::DifferentialDriveWheelSpeeds GetDifferentialWheelSpeeds();
  frc::MecanumDriveWheelSpeeds GetMecanumWheelSpeeds();

 private:
  frc::PWMSparkMax fl;
  frc::PWMSparkMax fr;
  frc::PWMSparkMax bl;
  frc::PWMSparkMax br;

  frc::Encoder flEncoder;
  frc::Encoder frEncoder;
  frc::Encoder blEncoder;
  frc::Encoder brEncoder;
  
  frc::AnalogGyro gyro;

  frc::sim::PWMSim s_fl;
  frc::sim::PWMSim s_fr;
  frc::sim::PWMSim s_bl;
  frc::sim::PWMSim s_br;

  frc::sim::EncoderSim s_flEncoder;
  frc::sim::EncoderSim s_frEncoder;
  frc::sim::EncoderSim s_blEncoder;
  frc::sim::EncoderSim s_brEncoder;

  frc::sim::AnalogGyroSim s_gyro;

  frc::Pose2d pose;
  frc::MecanumDriveOdometry odometry;

  frc::SimpleMotorFeedforward<units::meters> feedforward;

  frc::Field2d field;

  units::radians_per_second_t omega;
};
#endif
