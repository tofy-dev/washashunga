// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace Controller {
constexpr int lX_Joystick = 0;
constexpr int lY_Joystick = 1;

constexpr int lTrigger = 2;
constexpr int rTrigger = 3;

constexpr int rX_Joystick = 4;
constexpr int rY_Joystick = 5;
} // namespace Controller
