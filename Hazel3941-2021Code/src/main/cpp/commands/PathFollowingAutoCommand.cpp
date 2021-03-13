/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoCommands.h"

#include "Robot.h"

PathFollowingAutoCommand::PathFollowingAutoCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Drive);
}

// Called just before this Command runs the first time
void PathFollowingAutoCommand::Initialize() {
  timer = 100;
  Robot::navx.ZeroYaw();
}

// Called repeatedly when this Command is scheduled to run
void PathFollowingAutoCommand::Execute() {
  if(true/*timer > 0*/){
    Robot::Drive.driveControl.ArcadeDrive(0, -.3, false);
    timer--;
  }else{
    Robot::Drive.driveControl.ArcadeDrive(0, 0, false);
  }
  frc::SmartDashboard::PutNumber("NAVX", Robot::navx.GetRotation2d().Degrees().value());
}

// Make this return true when this Command no longer needs to run execute()
bool PathFollowingAutoCommand::IsFinished() { 
  if( abs(Robot::navx.GetRotation2d().Degrees().value() )> 90){
    // We turned our desired 90 degrees, so we are done!
    return true;
  } else {
    return false;
  }
 }

// Called once after isFinished returns true
void PathFollowingAutoCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PathFollowingAutoCommand::Interrupted() {}
