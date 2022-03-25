#include "RobotContainer.h"

#include "Constants.h"

#include <units/velocity.h>
#include <units/angular_velocity.h>

#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>

RobotContainer::RobotContainer() :
mecanumSubsystem{},

teleopCommand{&mecanumSubsystem, 0, 1},

trajectoryConfig{MAX_SPEED, MAX_ACCEL},
autoTrajectory{frc::TrajectoryGenerator::GenerateTrajectory(
  frc::Pose2d{0_m, 5_m, frc::Rotation2d(0_deg)},
  {frc::Translation2d{5_m, 2.5_m}, frc::Translation2d{10_m, 7.5_m}},
  frc::Pose2d{15_m, 5_m, frc::Rotation2d{0_deg}},

  trajectoryConfig
)},

autoCommand{
  autoTrajectory,
  [this]() { return mecanumSubsystem.GetPose(); },
  frc::RamseteController{ramseteB, ramseteZeta},
  // frc::SimpleMotorFeedforward<units::meters>{kS, kV, kA},
  // frc::SimpleMotorFeedforward<units::meters>{kS, kV, kA},
  // frc::SimpleMotorFeedforward<units::meters>{kS, kV, kA},
  frc::SimpleMotorFeedforward<units::meters>{0_V, 1_V * 1_s / 1_m},  
  frc::DifferentialDriveKinematics{units::meter_t{WIDTH}},
  [this] { return mecanumSubsystem.GetDifferentialWheelSpeeds(); },
  frc::PIDController{0, 0, 0},
  frc::PIDController{0, 0, 0},
  [this](auto l, auto r) {
    mecanumSubsystem.DriveVoltages(units::volt_t{l.value()}, units::volt_t{r.value()});
  },
  {&mecanumSubsystem}    
},

autoCommandGroup{std::move(autoCommand), frc2::InstantCommand{[this]{ mecanumSubsystem.DriveVoltages(0_V, 0_V); }, {&mecanumSubsystem}}}

{
  ConfigureButtonBindings();
  trajectoryConfig.SetKinematics(differentialKinematics);
  frc::DifferentialDriveVoltageConstraint voltageConstraint{feedforward, differentialKinematics, units::volt_t{12}};
  trajectoryConfig.AddConstraint(voltageConstraint);
}

void RobotContainer::ConfigureButtonBindings() {

}

frc2::Command* RobotContainer::GetTeleopCommand() {
  return &teleopCommand;
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return &autoCommandGroup;
}
