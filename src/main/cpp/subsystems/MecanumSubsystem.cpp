#include "subsystems/MecanumSubsystem.h"

#include "Constants.h"

#include <units/angle.h>

#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <algorithm>

; // i have no ida why i need this semicolon but this class WILL NOT compile without it
// update: im keeping it because i think its hilarious but there was no semicolon at the end of Constants.h
MecanumSubsystem::MecanumSubsystem() :
fl{FRONT_LEFT},
fr{FRONT_RIGHT},
bl{BACK_LEFT},
br{BACK_RIGHT},

flEncoder{FRONT_LEFT_ENCODER[0], FRONT_LEFT_ENCODER[1]},
frEncoder{FRONT_RIGHT_ENCODER[0], FRONT_RIGHT_ENCODER[1]},
blEncoder{BACK_LEFT_ENCODER[0], BACK_LEFT_ENCODER[1]},
brEncoder{BACK_RIGHT_ENCODER[0], BACK_RIGHT_ENCODER[1]},

gyro{GYRO},

s_fl{FRONT_LEFT},
s_fr{FRONT_RIGHT},
s_bl{BACK_LEFT},
s_br{BACK_RIGHT},

s_flEncoder{flEncoder},
s_frEncoder{frEncoder},
s_blEncoder{blEncoder},
s_brEncoder{brEncoder},

s_gyro{gyro},

pose{8_m, 2_m, frc::Rotation2d{0_deg}},
odometry{kinematics, units::degree_t{gyro.GetAngle()}, pose},

feedforward{kS, kV}
{
  fl.SetInverted(true);
  bl.SetInverted(true);

  flEncoder.SetDistancePerPulse(0.001);
  frEncoder.SetDistancePerPulse(0.001);
  blEncoder.SetDistancePerPulse(0.001);
  brEncoder.SetDistancePerPulse(0.001);

  flEncoder.SetReverseDirection(true);
  blEncoder.SetReverseDirection(true);
}

void MecanumSubsystem::Periodic() { 
  frc::MecanumDriveWheelSpeeds wheelSpeeds{units::meters_per_second_t{flEncoder.GetRate()},
    units::meters_per_second_t{frEncoder.GetRate()}, units::meters_per_second_t{blEncoder.GetRate()},
    units::meters_per_second_t{brEncoder.GetRate()}};
  odometry.Update(frc::Rotation2d{GetAngle()}, wheelSpeeds);  
  field.SetRobotPose(odometry.GetPose());
  frc::SmartDashboard::PutData("Field", &field);

  
}

void MecanumSubsystem::SimulationPeriodic() {

  double s_flPrev = s_flEncoder.GetDistance();
  double s_frPrev = s_frEncoder.GetDistance();
  double s_blPrev = s_blEncoder.GetDistance();
  double s_brPrev = s_brEncoder.GetDistance();

  s_flEncoder.SetDistance(0.02 * -s_fl.GetSpeed() * MAX_SPEED.value() + s_flEncoder.GetDistance());
  s_frEncoder.SetDistance(0.02 * s_fr.GetSpeed() * MAX_SPEED.value() + s_frEncoder.GetDistance());
  s_blEncoder.SetDistance(0.02 * -s_bl.GetSpeed() * MAX_SPEED.value() + s_blEncoder.GetDistance());
  s_brEncoder.SetDistance(0.02 * s_br.GetSpeed() * MAX_SPEED.value() + s_brEncoder.GetDistance());

  s_flEncoder.SetRate((s_flEncoder.GetDistance() - s_flPrev) / 0.02);
  s_frEncoder.SetRate((s_frEncoder.GetDistance() - s_frPrev) / 0.02);
  s_blEncoder.SetRate((s_blEncoder.GetDistance() - s_blPrev) / 0.02);
  s_brEncoder.SetRate((s_brEncoder.GetDistance() - s_brPrev) / 0.02);

  s_gyro.SetAngle(s_gyro.GetAngle() + kinematics.ToChassisSpeeds(frc::MecanumDriveWheelSpeeds{units::meters_per_second_t{s_flEncoder.GetRate()}, units::meters_per_second_t{s_frEncoder.GetRate()}, units::meters_per_second_t{s_blEncoder.GetRate()}, units::meters_per_second_t{s_brEncoder.GetRate()}}).omega.value() * (180/3.14159) * 0.02);
}

void MecanumSubsystem::Drive(units::meters_per_second_t vx, units::meters_per_second_t vy, units::radians_per_second_t omega) {
  frc::MecanumDriveWheelSpeeds wheelSpeeds = kinematics.ToWheelSpeeds(frc::ChassisSpeeds{vy, vx, omega});

  wheelSpeeds.Desaturate(MAX_SPEED);

  DriveVoltages(feedforward.Calculate(units::meters_per_second_t{std::clamp(wheelSpeeds.frontLeft.value(), -MAX_SPEED.value(), MAX_SPEED.value())}),
   feedforward.Calculate(units::meters_per_second_t{std::clamp(wheelSpeeds.frontRight.value(), -MAX_SPEED.value(), MAX_SPEED.value())}),
   feedforward.Calculate(units::meters_per_second_t{std::clamp(wheelSpeeds.rearLeft.value(), -MAX_SPEED.value(), MAX_SPEED.value())}),
   feedforward.Calculate(units::meters_per_second_t{std::clamp(wheelSpeeds.rearRight.value(), -MAX_SPEED.value(), MAX_SPEED.value())}));

  // MecanumSubsystem::omega = omega;
}

void MecanumSubsystem::DriveVoltages(units::volt_t _fl, units::volt_t _fr, units::volt_t _bl, units::volt_t _br) {
  fl.SetVoltage(_fl);
  fr.SetVoltage(_fr);
  bl.SetVoltage(_bl);
  br.SetVoltage(_br);

  s_fl.SetSpeed(std::clamp(_fl.value(), -12.0, 12.0) / -12);
  s_fr.SetSpeed(std::clamp(_fr.value(), -12.0, 12.0) / 12);
  s_bl.SetSpeed(std::clamp(_bl.value(), -12.0, 12.0) / -12);
  s_br.SetSpeed(std::clamp(_br.value(), -12.0, 12.0) / 12);
}

void MecanumSubsystem::Drive(units::meters_per_second_t vx, units::radians_per_second_t omega) {
  Drive(vx, 0_mps, omega);
}

void MecanumSubsystem::DriveVoltages(units::volt_t left, units::volt_t right) {
  DriveVoltages(left, right, left, right);
}

void MecanumSubsystem::ResetEncoders() {
  flEncoder.Reset();
  frEncoder.Reset();
  blEncoder.Reset();
  brEncoder.Reset();
}

units::degree_t MecanumSubsystem::GetAngle() {
  return units::degree_t{GetAngleRaw()};
}

double MecanumSubsystem::GetAngleRaw() {
  return gyro.GetAngle(); 
}

frc::Pose2d MecanumSubsystem::GetPose() {
  return odometry.GetPose();
}

frc::MecanumDriveWheelSpeeds MecanumSubsystem::GetMecanumWheelSpeeds() {
  return {units::meters_per_second_t{flEncoder.GetRate()}, units::meters_per_second_t{frEncoder.GetRate()}, units::meters_per_second_t{blEncoder.GetRate()}, units::meters_per_second_t{brEncoder.GetRate()}};
}

frc::DifferentialDriveWheelSpeeds MecanumSubsystem::GetDifferentialWheelSpeeds() {
  return {units::meters_per_second_t{flEncoder.GetRate()}, units::meters_per_second_t{frEncoder.GetRate()}};
}

void MecanumSubsystem::SetPose(frc::Pose2d _pose) {
  pose = _pose;
  odometry = frc::MecanumDriveOdometry{kinematics, _pose.Rotation().Degrees(), _pose};
}