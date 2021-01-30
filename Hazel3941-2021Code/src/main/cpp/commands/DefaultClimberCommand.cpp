/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Commands.h"

#include "Robot.h"

DefaultClimberCommand::DefaultClimberCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Climber);
}

// Called just before this Command runs the first time
void DefaultClimberCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultClimberCommand::Execute() {
  if(Robot::oi.OperatorController->GetRawButton(OPERATOR_CLIMB_UP_BUTTON)){
    Robot::Climber.climber.Set(motorcontrol::ControlMode::PercentOutput, 1.0);
  }else if(Robot::oi.OperatorController->GetRawButton(OPERATOR_CLIMB_RELEASE_BUTTON)){
    Robot::Climber.climber.Set(motorcontrol::ControlMode::PercentOutput, -1.0);
  }else{
    Robot::Climber.climber.Set(motorcontrol::ControlMode::PercentOutput, 0.0);
  }
  if(Robot::oi.OperatorController->GetRawButton(OPERATOR_LIFT_UP_BUTTON)){
    Robot::Climber.lift.Set(motorcontrol::ControlMode::PercentOutput, 1.0);
  }else if(Robot::oi.OperatorController->GetRawButton(OPERATOR_LIFT_DOWN_BUTTON)){
    Robot::Climber.lift.Set(motorcontrol::ControlMode::PercentOutput, -1.0);
  } else {
    Robot::Climber.lift.Set(motorcontrol::ControlMode::PercentOutput, 0.0);
  }
}

// Make this return true when this Command no longer needs to run execute()
bool DefaultClimberCommand::IsFinished() { return false; }

// Called once after isFinished returns true
void DefaultClimberCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DefaultClimberCommand::Interrupted() {}
