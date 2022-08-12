#pragma once

#include <frc2/command/Command.h>

#include "subsystems/MecanumSubsystem.h"

#include "commands/TeleopCommand.h"
#include "commands/TurnCommand.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>


#include "Constants.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetTeleopCommand();
  frc2::Command* GetAutonomousCommand();

 private:
  MecanumSubsystem mecanumSubsystem;


  TeleopCommand teleopCommand;
  TurnCommand testTurnCommand;

  frc::TrajectoryConfig trajectoryConfig;
  frc::Trajectory autoTrajectory;
  frc2::RamseteCommand autoCommand;
  frc2::SequentialCommandGroup autoCommandGroup;

  const frc::SimpleMotorFeedforward<units::meters> feedforward{kS, kV, kA};

  void ConfigureButtonBindings();
};
