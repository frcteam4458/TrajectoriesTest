#include "RobotContainer.h"

#include "Constants.h"

#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>

#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

RobotContainer::RobotContainer() :
mecanumSubsystem{},

teleopCommand{&mecanumSubsystem, 0, 1},
testTurnCommand{&mecanumSubsystem, 90},

trajectoryConfig{MAX_SPEED, MAX_ACCEL},

// autoTrajectory{frc::TrajectoryGenerator::GenerateTrajectory(
//   frc::Pose2d{0_m, 5_m, frc::Rotation2d(0_deg)},
//   {
//     frc::Translation2d{1_m, 4.75_m},
//     frc::Translation2d{2_m, 4.5_m},
//     frc::Translation2d{3_m, 4.25_m},
//     frc::Translation2d{4_m, 4_m},
//     frc::Translation2d{5_m, 4.25_m},
//     frc::Translation2d{6_m, 4.5_m},
//     frc::Translation2d{7_m, 4.75_m},
//     frc::Translation2d{8_m, 6_m},
//     frc::Translation2d{9_m, 5_m},
//     frc::Translation2d{10_m, 5_m},
//     frc::Translation2d{11_m, 5_m},
//     frc::Translation2d{12_m, 5_m},
//     frc::Translation2d{13_m, 5_m},
//     frc::Translation2d{14_m, 5_m}
//   },
//   frc::Pose2d{15_m, 5_m, frc::Rotation2d{0_deg}},

//   trajectoryConfig
// )},

// autoTrajectory{frc::TrajectoryGenerator::GenerateTrajectory(
//   frc::Pose2d{7_m, 4.5_m, frc::Rotation2d{0_deg}},
//   {
//     frc::Translation2d{5.7_m, 5.7_m},
//     frc::Translation2d{6_m, 5_m},
//     frc::Translation2d{7.6_m, 7.3_m},
//     frc::Translation2d{5.6_m, 2.7_m},
//     frc::Translation2d{7.9_m, 1.2_m}
//   },
//   frc::Pose2d{7.7_m, 2_m, frc::Rotation2d{180_deg}},
//   trajectoryConfig
// )},

autoTrajectory(frc::TrajectoryUtil::FromPathweaverJson(frc::filesystem::GetDeployDirectory() + "/Auto2.wpilib.json")),

autoCommand{
  // autoTrajectory,
  frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}},

    {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},

    frc::Pose2d{3_m, 0_m, frc::Rotation2d{0_deg}},

    trajectoryConfig
  ),

  [this]() { return mecanumSubsystem.GetPose(); },
  frc::RamseteController{ramseteB, ramseteZeta},
  frc::SimpleMotorFeedforward<units::meters>{kS, kV},  
  frc::DifferentialDriveKinematics{units::meter_t{WIDTH}},
  [this]() { return mecanumSubsystem.GetDifferentialWheelSpeeds(); },
  frc::PIDController{0, 0, 0},
  frc::PIDController{0, 0, 0},
  [this](auto l, auto r) {
    
    // mecanumSubsystem.DriveVoltages(units::volt_t{l.value()}, units::volt_t{r.value()});
  },
  {&mecanumSubsystem}    
},

autoCommandGroup{frc2::InstantCommand{[this]{ mecanumSubsystem.SetPose(autoTrajectory.InitialPose()); }, {&mecanumSubsystem}}, std::move(autoCommand), frc2::InstantCommand{[this]{ mecanumSubsystem.DriveVoltages(0_V, 0_V); }, {&mecanumSubsystem}}}

{
  ConfigureButtonBindings();
  autoTrajectory.Sample(0_s);
  // trajectoryConfig.SetKinematics(differentialKinematics);
  // frc::DifferentialDriveVoltageConstraint voltageConstraint{feedforward, differentialKinematics, units::volt_t{12}};
  // trajectoryConfig.AddConstraint(voltageConstraint);

}

void RobotContainer::ConfigureButtonBindings() {

}

frc2::Command* RobotContainer::GetTeleopCommand() {
  return &teleopCommand;
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  auto initialState = autoTrajectory.Sample(0_s);
  auto m_prevSpeeds = kinematics.ToWheelSpeeds(frc::ChassisSpeeds{initialState.velocity, 0_mps, initialState.velocity * initialState.curvature});
  
  // autoCommand.Schedule();
  // autoCommand.Initialize();
  return &autoCommand;
}