#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <frc/Joystick.h>

#include "subsystems/MecanumSubsystem.h"

class TeleopCommand : public frc2::CommandHelper<frc2::CommandBase, TeleopCommand> {
public:
    explicit TeleopCommand(MecanumSubsystem* _subsystem, int leftStick, int rightStick);
    void Execute() override;
    void End(bool interrupted) override;
private:
    MecanumSubsystem* subsystem;

    frc::Joystick left;
    frc::Joystick right;
};
