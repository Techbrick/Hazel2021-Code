/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Commands.h"

#include "Robot.h"
#include "RobotMap.h"

DefaultIntakeCommand::DefaultIntakeCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Intake);
}

// Called just before this Command runs the first time
void DefaultIntakeCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultIntakeCommand::Execute() {
  /*if(Robot::oi.DriverController->GetRawButton(INTAKE_OFF_BUTTON)){
    Robot::Intake.manualEnabled = false;
  }else if(Robot::oi.DriverController->GetRawButton(INTAKE_ON_BUTTON)){
    Robot::Intake.manualEnabled = true;
  }*/

  if(!Robot::oi.OperatorController->GetRawButton(MANUAL_OPERATOR_OVERRIDE_BUTTON)){
    if(Robot::oi.DriverController->GetRawButton(DRIVE_CONTROLLER_ENABLE_INTAKEINDEXING_BUTTON)){
      if(!Robot::Intake.buttonLastA){
        Robot::Intake.manualEnabled = !Robot::Intake.manualEnabled;
      }
      Robot::Intake.buttonLastA = true;
    }else{
      Robot::Intake.buttonLastA = false;
    }
    if(Robot::Intake.manualEnabled && Robot::Intake.indexEnabled){
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, -0.5);
    }else{
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
    }
    if(Robot::oi.DriverController->GetRawButton(DRIVE_CONTROLLER_EXTEND_RETRACT_INTAKE_BUTTON)){
      if(!Robot::Intake.buttonLastB){
        Robot::Intake.extended = !Robot::Intake.extended;
      }
      Robot::Intake.buttonLastB = true;
    }else{
      Robot::Intake.buttonLastB = false;
    }
    if(!Robot::Intake.extended){
      Robot::Intake.ExtenderSolenoidA.Set(frc::DoubleSolenoid::Value::kForward);
      Robot::Intake.ExtenderSolenoidB.Set(frc::DoubleSolenoid::Value::kForward);
    }else{
      Robot::Intake.ExtenderSolenoidA.Set(frc::DoubleSolenoid::Value::kReverse);
      Robot::Intake.ExtenderSolenoidB.Set(frc::DoubleSolenoid::Value::kReverse);
    }
  }else{
    // operator is overriding the mechanism
    if(Robot::oi.OperatorController->GetRawButton(OPERATOR_INTAKE_FEED_ROBOT_BUTTON)){
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, 0.5);
    }else if(Robot::oi.OperatorController->GetRawButton(OPERATOR_INTAKE_REJECT_BUTTON)){
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, -0.5);
    }else{
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
    }
  }
}

// Make this return true when this Command no longer needs to run execute()
bool DefaultIntakeCommand::IsFinished() { return false; }

// Called once after isFinished returns true
void DefaultIntakeCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DefaultIntakeCommand::Interrupted() {}
