/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Commands.h"

#include "Robot.h"

TestTrajectoryCommand::TestTrajectoryCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Drive);
}

// Called just before this Command runs the first time
void TestTrajectoryCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TestTrajectoryCommand::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool TestTrajectoryCommand::IsFinished() { return false; }

// Called once after isFinished returns true
void TestTrajectoryCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TestTrajectoryCommand::Interrupted() {}
