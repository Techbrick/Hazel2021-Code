/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Commands.h"

#include "Robot.h"

trackCommand::trackCommand() {
  // Use Requires() here to declare subsystem dependencies
  //Requires(&Robot::Indexer);
  Requires(&Robot::Drive);
  //Requires(&Robot::Shooter);
}

// Called just before this Command runs the first time
void trackCommand::Initialize() {
  frc::SmartDashboard::PutNumber("Track P", PC);
  frc::SmartDashboard::PutNumber("Track I", IC);
  frc::SmartDashboard::PutNumber("Track D", DC);
}

// Called repeatedly when this Command is scheduled to run
void trackCommand::Execute() {
  float lastTx = tx;
  tx = Robot::table->GetNumber("tx", 0.0);
  ty = Robot::table->GetNumber("ty", 0.0);
  frc::SmartDashboard::PutNumber("txDebug", tx);
  frc::SmartDashboard::PutNumber("tyDebug", ty);
  PC = frc::SmartDashboard::GetNumber("Track P", 0.0);
  IC = frc::SmartDashboard::GetNumber("Track I", 0.0);
  DC = frc::SmartDashboard::GetNumber("Track D", 0.0);
  P = tx;
  I = .9 * tx + .1 * I;
  D = lastTx - tx;
  float PID = PC * P + IC * I +  DC * D;
  Robot::Drive.driveControl.ArcadeDrive(DRIVE_CONTROLLER_DRIVE_AXIS_X, -PID);
}

// Make this return true when this Command no longer needs to run execute()
bool trackCommand::IsFinished() { return !Robot::oi.DriverController->GetRawButton(DRIVE_TRACK_BUTTON); }

// Called once after isFinished returns true
void trackCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void trackCommand::Interrupted() {}
