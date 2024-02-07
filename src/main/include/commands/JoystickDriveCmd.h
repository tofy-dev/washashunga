// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/DriveSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class JoystickDriveCmd : public frc2::CommandHelper<frc2::Command, JoystickDriveCmd> {
 public:

  JoystickDriveCmd(DriveSubsystem* subsystem, 
    std::function<double()> xSpeedFcn, 
    std::function<double()> ySpeedFcn,
    std::function<bool()> turnInPlaceFcn);

  void Execute();

 private:
  DriveSubsystem* m_subsystem;

  std::function<double()> xSpeedFcn, ySpeedFcn;
  std::function<bool()> turnInPlaceFcn;
};
