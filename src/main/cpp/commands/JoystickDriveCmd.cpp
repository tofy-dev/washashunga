// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/JoystickDriveCmd.h"

JoystickDriveCmd::JoystickDriveCmd(DriveSubsystem *subsystem,
                                   std::function<double()> xSpeedFcn,
                                   std::function<double()> ySpeedFcn,
                                   std::function<bool()> turnInPlaceFcn)
    : m_subsystem{subsystem}, xSpeedFcn{xSpeedFcn}, ySpeedFcn{ySpeedFcn},
      turnInPlaceFcn{turnInPlaceFcn} {
  // Register that this command requires the subsystem.
  AddRequirements(m_subsystem);
}

void JoystickDriveCmd::Execute() {
  double xSpeed = xSpeedFcn();
  double ySpeed = ySpeedFcn();

  if (xSpeed < 0) {
    ySpeed = -ySpeed;
  }

  m_subsystem->JoystickDrive(xSpeed, ySpeed, turnInPlaceFcn());
}