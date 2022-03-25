#include "RobotContainer.h"

#include <frc2/command/RamseteCommand.h>

RobotContainer::RobotContainer() :
mecanumSubsystem{},

teleopCommand{&mecanumSubsystem, 0, 1}

{
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // mecanumSubsystem.SetDefaultCommand(std::move(teleopCommand));
}

frc2::Command* RobotContainer::GetTeleopCommand() {
  return &teleopCommand;
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return nullptr;
}
