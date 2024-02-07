// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include "Constants.h"
#include "commands/JoystickDriveCmd.h"
#include "commands/RunConveyorCmd.h"

using namespace Controller;
RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_drive.SetDefaultCommand(JoystickDriveCmd{
      &m_drive,
      [&]() -> double {
        return driveController.GetRawAxis(rTrigger) -
               driveController.GetRawAxis(lTrigger);
      },
      [&]() -> double { return driveController.GetRawAxis(lX_Joystick); },
      [&]() -> bool { return driveController.GetRawButton(1); }});

  m_conveyor.SetDefaultCommand(
      RunConveyorCmd{&m_conveyor, [&]() -> double {
                       return opController.GetRawAxis(rTrigger) -
                              opController.GetRawAxis(lTrigger);
                     }});

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  driveController.Button(1).WhileTrue(
      m_drive.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  driveController.Button(2).WhileTrue(
      m_drive.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  driveController.Button(3).WhileTrue(
      m_drive.SysIdDynamic(frc2::sysid::Direction::kForward));
  driveController.Button(4).WhileTrue(
      m_drive.SysIdDynamic(frc2::sysid::Direction::kReverse));
}
