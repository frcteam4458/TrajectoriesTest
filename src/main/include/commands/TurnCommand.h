#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Joystick.h>

#include "subsystems/MecanumSubsystem.h"

#include <frc/controller/PIDController.h>

class TurnCommand : public frc2::CommandHelper<frc2::CommandBase, TurnCommand> {
public:
    explicit TurnCommand(MecanumSubsystem* _subsystem, double _angle);
    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;
private:
    MecanumSubsystem* subsystem;

    frc::PIDController controller;

    double angle;
    double turnTo = 0;
};
