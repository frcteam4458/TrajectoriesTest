#include "RobotContainer.h"

#include "Constants.h"

#include <units/velocity.h>
#include <units/angular_velocity.h>



RobotContainer::RobotContainer() :
mecanumSubsystem{},

teleopCommand{&mecanumSubsystem, 0, 1},

trajectoryConfig{MAX_SPEED, MAX_ACCEL},
autoTrajectory{frc::TrajectoryGenerator::GenerateTrajectory(
  frc::Pose2d{0_m, 0_m, frc::Rotation2d(0_deg)},

  {frc::Translation2d{5_m, 5_m}, frc::Translation2d{10_m, -5_m}},

  frc::Pose2d{3_m, 0_m, frc::Rotation2d{0_deg}},

  trajectoryConfig
)},

autoCommand{
  autoTrajectory,
  [this]() { return mecanumSubsystem.GetPose(); },
  frc::RamseteController{ramseteB, ramseteZeta},
  // frc::SimpleMotorFeedforward<units::meters>{kS, kV, kA},
  // frc::SimpleMotorFeedforward<units::meters>{kS, kV, kA},
  // frc::SimpleMotorFeedforward<units::meters>{kS, kV, kA},
  frc::SimpleMotorFeedforward<units::meters>{kS, kV, kA},  
  frc::DifferentialDriveKinematics{units::meter_t{WIDTH}},
  [this] { return mecanumSubsystem.GetDifferentialWheelSpeeds(); },
  frc::PIDController{0, 0, 0},
  frc::PIDController{0, 0, 0},
  [this](auto l, auto r) {
    mecanumSubsystem.DriveVoltages(l, r);
  },
  {&mecanumSubsystem}    
}

{
  ConfigureButtonBindings();
  trajectoryConfig.SetKinematics(kinematics);
}

void RobotContainer::ConfigureButtonBindings() {
  // mecanumSubsystem.SetDefaultCommand(std::move(teleopCommand));
}

frc2::Command* RobotContainer::GetTeleopCommand() {
  return &teleopCommand;
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return &autoCommand;
}
