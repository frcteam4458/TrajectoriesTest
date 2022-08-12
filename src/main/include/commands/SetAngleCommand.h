#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Joystick.h>

#include "subsystems/MecanumSubsystem.h"

class SetAngleCommand : public frc2::CommandHelper<frc2::CommandBase, SetAngleCommand> {
public:
    explicit SetAngleCommand(MecanumSubsystem* _subsystem, double _angle);
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;
private:
    MecanumSubsystem* subsystem;

    double angle;
};
