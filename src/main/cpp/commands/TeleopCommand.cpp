#include "commands/TeleopCommand.h"

#include "Constants.h"

TeleopCommand::TeleopCommand(MecanumSubsystem* _subsystem, int leftStick, int rightStick) :
subsystem{_subsystem},
left{leftStick},
right{rightStick}
{
    AddRequirements(subsystem);
}

void TeleopCommand::Execute() {

    double x = left.GetX();
    double y = left.GetY();
    // double omega = right.GetX();
    double omega = left.GetRawAxis(2);

    subsystem->Drive(units::meters_per_second_t{-y * MAX_SPEED.value()}, units::radians_per_second_t{x * MAX_ROT_SPEED.value()});
}

void TeleopCommand::End(bool interrupted) {
    subsystem->DriveVoltages(0_V, 0_V, 0_V, 0_V);
}