#include "subsystems/MecanumSubsystem.h"

#include "Constants.h"
#include <wpi/math>

#include <units/angle.h>

#include <frc/kinematics/MecanumDriveWheelSpeeds.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <algorithm>

; // i have no ida why i need this semicolon but this class WILL NOT compile without it
// update: im keeping it because i think its hilarious but there was no semicolon at the end of Constants.h

// staying named mecanum but changing it to diffrential
MecanumSubsystem::MecanumSubsystem() :
drivetrainSim{
  frc::DCMotor::CIM(2),
  7.29,
  7.5_kg_sq_m,
  60_kg,
  3_in,
  0.7112_m,

  {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}
},   // i stole all of these measurements from the docs, just changed neo to cim

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
odometry{units::degree_t{gyro.GetAngle()}, pose},



feedforward{kS, kV}
{
  fl.SetInverted(true);
  bl.SetInverted(true);

  s_flEncoder.SetCount(1000);
  s_frEncoder.SetCount(1000);
  s_blEncoder.SetCount(1000);
  s_brEncoder.SetCount(1000);

  // 0.0762 is the wheel radius in meters
  flEncoder.SetDistancePerPulse(2 * wpi::math::pi * 0.0762 / s_flEncoder.GetCount());
  frEncoder.SetDistancePerPulse(2 * wpi::math::pi * 0.0762 / s_frEncoder.GetCount());
  blEncoder.SetDistancePerPulse(2 * wpi::math::pi * 0.0762 / s_blEncoder.GetCount());
  brEncoder.SetDistancePerPulse(2 * wpi::math::pi * 0.0762 / s_brEncoder.GetCount());

  flEncoder.SetReverseDirection(true);
  blEncoder.SetReverseDirection(true);
}

void MecanumSubsystem::Periodic() { 
  frc::DifferentialDriveWheelSpeeds wheelSpeeds{units::meters_per_second_t{flEncoder.GetRate()},
    units::meters_per_second_t{frEncoder.GetRate()}};
  odometry.Update(frc::Rotation2d{GetAngle()}, units::meter_t{flEncoder.GetDistance()}, units::meter_t{frEncoder.GetDistance()});  
  field.SetRobotPose(odometry.GetPose());
  frc::SmartDashboard::PutData("Field", &field);
}

void MecanumSubsystem::SimulationPeriodic() {
  drivetrainSim.SetInputs(fl.Get() * 12_V, fr.Get() * 12_V);

  drivetrainSim.Update(20_ms);

  s_flEncoder.SetDistance(drivetrainSim.GetLeftPosition().value());
  s_flEncoder.SetRate(drivetrainSim.GetLeftVelocity().value());
  s_frEncoder.SetDistance(drivetrainSim.GetLeftPosition().value());
  s_frEncoder.SetRate(drivetrainSim.GetLeftVelocity().value());
  s_blEncoder.SetDistance(drivetrainSim.GetRightPosition().value());
  s_blEncoder.SetRate(drivetrainSim.GetRightPosition().value());
  s_brEncoder.SetDistance(drivetrainSim.GetRightPosition().value());
  s_brEncoder.SetRate(drivetrainSim.GetRightPosition().value());

  s_gyro.SetRate((-drivetrainSim.GetHeading().Degrees().value() - -drivetrainSim.GetHeading().Degrees().value()) / 0.02);
  s_gyro.SetAngle(-drivetrainSim.GetHeading().Degrees().value());
}

void MecanumSubsystem::Drive(units::meters_per_second_t vy, units::radians_per_second_t omega) {
  frc::DifferentialDriveWheelSpeeds wheelSpeeds = differentialKinematics.ToWheelSpeeds(frc::ChassisSpeeds{vy, 0_mps, omega});

  wheelSpeeds.Desaturate(MAX_SPEED);

  DriveVoltages(feedforward.Calculate(units::meters_per_second_t{std::clamp(wheelSpeeds.left.value(), -MAX_SPEED.value(), MAX_SPEED.value())}),
   feedforward.Calculate(units::meters_per_second_t{std::clamp(wheelSpeeds.right.value(), -MAX_SPEED.value(), MAX_SPEED.value())}));
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

void MecanumSubsystem::DriveVoltages(units::volt_t left, units::volt_t right) {
  DriveVoltages(left, right, left, right);
}

void MecanumSubsystem::Turn(double speed) {
  DriveVoltages(feedforward.Calculate(units::meters_per_second_t{speed}), -feedforward.Calculate(units::meters_per_second_t{speed}));
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
  return gyro.GetAngle() + gyroOffset; 
}

frc::Pose2d MecanumSubsystem::GetPose() {
  return odometry.GetPose();
}

frc::DifferentialDriveWheelSpeeds MecanumSubsystem::GetMecanumWheelSpeeds() {
  return {units::meters_per_second_t{flEncoder.GetRate()}, units::meters_per_second_t{frEncoder.GetRate()}};
}

frc::DifferentialDriveWheelSpeeds MecanumSubsystem::GetDifferentialWheelSpeeds() {
  return frc::DifferentialDriveWheelSpeeds{units::meters_per_second_t{flEncoder.GetRate()}, units::meters_per_second_t{frEncoder.GetRate()}};
}

void MecanumSubsystem::SetPose(frc::Pose2d _pose) {
  pose = _pose;
  odometry = frc::DifferentialDriveOdometry{_pose.Rotation().Degrees(), _pose};
  gyro.Reset();
  gyroOffset = _pose.Rotation().Degrees().value();
}