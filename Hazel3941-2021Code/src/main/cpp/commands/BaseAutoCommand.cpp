/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoCommands.h"

#include "Robot.h"

baseAutoCommand::baseAutoCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::m_subsystem);
}

// Called just before this Command runs the first time
void baseAutoCommand::Initialize() {
  timer = 100;
}

// Called repeatedly when this Command is scheduled to run
void baseAutoCommand::Execute() {
  if(timer > 0){
    Robot::Drive.driveControl.ArcadeDrive(0.5, 0, false);
    timer--;
  }else{
    Robot::Drive.driveControl.ArcadeDrive(0, 0, false);
  }
}

// Make this return true when this Command no longer needs to run execute()
bool baseAutoCommand::IsFinished() { return !(timer > 0); }

// Called once after isFinished returns true
void baseAutoCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void baseAutoCommand::Interrupted() {}
