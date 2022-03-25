#pragma once

#include <frc2/command/Command.h>

#include "commands/TeleopCommand.h"
#include "subsystems/MecanumSubsystem.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetTeleopCommand();
  frc2::Command* GetAutonomousCommand();

 private:
  MecanumSubsystem mecanumSubsystem;


  TeleopCommand teleopCommand;


  frc::TrajectoryConfig trajectoryConfig;
  frc::Trajectory autoTrajectory;
  frc2::RamseteCommand autoCommand;
  // frc2::SequentialCommandGroup autoCommandGroup;

  void ConfigureButtonBindings();
};
