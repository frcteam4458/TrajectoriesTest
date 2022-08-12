#include "commands/SetAngleCommand.h"

#include "Constants.h"
#include "MathUtil.h"

SetAngleCommand::SetAngleCommand(MecanumSubsystem* _subsystem, double _angle) :
subsystem{_subsystem},
angle(_angle)
{
    AddRequirements(subsystem);
}

void SetAngleCommand::Execute() {
    subsystem->Turn(AngleDifference(subsystem->GetAngle().value(), angle) * 0.02 * MAX_SPEED.value());
}

bool SetAngleCommand::IsFinished() {
    return abs(AngleDifference(subsystem->GetAngle().value(), angle)) < 1;
}

void SetAngleCommand::End(bool interrupted) {
    subsystem->DriveVoltages(0_V, 0_V, 0_V, 0_V);
}