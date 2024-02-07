// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/ConveyorSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class RunConveyorCmd : public frc2::CommandHelper<frc2::Command, RunConveyorCmd> {
 public:

  RunConveyorCmd(ConveyorSubsystem* subsystem, 
    std::function<double()> speedFcn);

  void Execute();

 private:
  ConveyorSubsystem* m_subsystem;
  std::function<double()> speedFcn;
};
