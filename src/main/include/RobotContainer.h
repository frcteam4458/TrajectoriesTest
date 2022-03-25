#pragma once

#include <frc2/command/Command.h>

#include "commands/TeleopCommand.h"
#include "subsystems/MecanumSubsystem.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetTeleopCommand();
  frc2::Command* GetAutonomousCommand();

 private:
  MecanumSubsystem mecanumSubsystem;


  TeleopCommand teleopCommand;

  void ConfigureButtonBindings();
};
