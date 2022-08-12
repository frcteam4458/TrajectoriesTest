#include "commands/TurnCommand.h"

#include "Constants.h"
#include "MathUtil.h"

#include <frc/controller/PIDController.h>

#include <frc/smartdashboard/SmartDashboard.h>

TurnCommand::TurnCommand(MecanumSubsystem* _subsystem, double _angle) :
subsystem{_subsystem},
angle{_angle},
controller{0.0001, 0.001, 0.001}
{
    controller.SetSetpoint(90);
    controller.EnableContinuousInput(0, 360);
    controller.SetTolerance(5, 20);
    AddRequirements(subsystem);
}

void TurnCommand::Initialize() {
    // turnTo = angle + subsystem->GetAngle().value();

}

void TurnCommand::Execute() {
    double output = -controller.Calculate(-subsystem->GetAngle().value());
    subsystem->Turn(output);
    frc::SmartDashboard::PutNumber("Output", output);
}

bool TurnCommand::IsFinished() {
    return controller.AtSetpoint();
}

void TurnCommand::End(bool interrupted) {
    subsystem->DriveVoltages(0_V, 0_V, 0_V, 0_V);
}