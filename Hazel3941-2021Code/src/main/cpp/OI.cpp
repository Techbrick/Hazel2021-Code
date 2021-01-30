/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "OI.h"
#include "RobotMap.h"
#include "commands/Commands.h"
#include <frc/buttons/JoystickButton.h>

OI::OI() {
  // Process operator interface input here.
  DriverController = new frc::Joystick(0);
  OperatorController = new frc::Joystick(1);
	frc::JoystickButton* trackButton = new frc::JoystickButton(DriverController, DRIVE_TRACK_BUTTON);
  trackButton->WhenPressed(new trackCommand());
}