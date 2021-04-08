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

  float speedFactor = .75;
  if(Robot::oi.DriverController->GetRawButton(DRIVE_TOGGLE_AUTO_INTAKE_BUTTON)){
    if(!Robot::Intake.buttonLastA){
      // on flip of button
      Robot::Intake.autoIntake = !Robot::Intake.autoIntake;
      //Robot::Intake.manualEnabled = !Robot::Intake.manualEnabled;
    }
    Robot::Intake.buttonLastA = true;
  }else{
    Robot::Intake.buttonLastA = false;
  }

  if(Robot::oi.DriverController->GetRawButton(DRIVE_CONTROLLER_EXTEND_RETRACT_INTAKE_BUTTON)){
    if(!Robot::Intake.buttonLastB){
      // on flip of button
      Robot::Intake.extended = !Robot::Intake.extended;
    }
    Robot::Intake.buttonLastB = true;
  }else{
    Robot::Intake.buttonLastB = false;
  }


  if(Robot::oi.OperatorController->GetRawButton(MANUAL_OPERATOR_OVERRIDE_BUTTON)){
    Robot::Intake.expell = Robot::oi.OperatorController->GetRawButton(OPERATOR_INTAKE_REJECT_BUTTON);
    Robot::Intake.backDriveBelt = Robot::oi.OperatorController->GetRawButton(OPERATOR_INDEXER_FEED_REVERSE_BUTTON);
    Robot::Intake.backDriveIndex = Robot::Intake.backDriveBelt;
    Robot::Intake.beltOn = Robot::Intake.backDriveBelt || Robot::oi.OperatorController->GetRawButton(OPERATOR_INDEXER_FEED_FORWARD_BUTTON);
    Robot::Intake.intakeWheelOn = Robot::Intake.expell || Robot::oi.OperatorController->GetRawButton(OPERATOR_INTAKE_FEED_ROBOT_BUTTON);
    Robot::Intake.indexWheelOn = Robot::Intake.beltOn;

  }else{
    if(Robot::Intake.autoIntake){

      // <AUTO>
      
      Robot::Intake.extended = true;

      switch(state){
        case 0:
          Robot::Intake.beltOn = false;
          Robot::Intake.intakeWheelOn = true;
          Robot::Intake.indexWheelOn = false;
          Robot::Intake.expell = false;
          Robot::Intake.backDriveBelt = false;
          Robot::Intake.backDriveIndex = false;
          if(!Robot::Intake.distanceA.Get() && !Robot::Intake.distanceB.Get()){
            state++;
          }
          break;
        case 1:
          Robot::Intake.beltOn = true;
          Robot::Intake.intakeWheelOn = true;
          Robot::Intake.indexWheelOn = false;
          Robot::Intake.expell = false;
          Robot::Intake.backDriveBelt = false;
          Robot::Intake.backDriveIndex = false;
          if(!Robot::Intake.beltA.Get()){
            state++;
          }
          break;
        case 2:
          Robot::Intake.beltOn = true;
          Robot::Intake.intakeWheelOn = true;
          Robot::Intake.indexWheelOn = true;
          Robot::Intake.expell = true;
          Robot::Intake.backDriveBelt = false;
          Robot::Intake.backDriveIndex = false;
          if(Robot::Intake.beltA.Get()){
            state = 0;
          }
          break;
      }

      // </AUTO>

    }else{

      Robot::Intake.beltOn = false;
      Robot::Intake.intakeWheelOn = false;
      Robot::Intake.indexWheelOn = false;
      Robot::Intake.expell = false;
      Robot::Intake.backDriveBelt = false;
      Robot::Intake.backDriveIndex = false;

    }
  }


  if(!Robot::Intake.extended){
    Robot::Intake.ExtenderSolenoidA.Set(frc::DoubleSolenoid::Value::kForward);
    Robot::Intake.ExtenderSolenoidB.Set(frc::DoubleSolenoid::Value::kForward);
  }else{
    Robot::Intake.ExtenderSolenoidA.Set(frc::DoubleSolenoid::Value::kReverse);
    Robot::Intake.ExtenderSolenoidB.Set(frc::DoubleSolenoid::Value::kReverse);
  }

  if(Robot::Intake.indexWheelOn){
    if(Robot::Intake.backDriveIndex){
      Robot::Intake.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, -0.45);
    }else{
      Robot::Intake.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 0.45);
    }
    
  }else{
    Robot::Intake.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
  }

  if(Robot::Intake.beltOn){
    if(Robot::Intake.backDriveBelt){
      Robot::Intake.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, -0.45);
    }else{
      Robot::Intake.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, speedFactor);
    }
  }else{
    Robot::Intake.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
  }

  if(Robot::Intake.intakeWheelOn){
    if(Robot::Intake.expell){
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, 0.5);
    }else{
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, -0.5);
    }
  }else{
    Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
  }

  Robot::Intake.lastStates[0] = Robot::Intake.distanceA.Get();
	Robot::Intake.lastStates[1] = Robot::Intake.distanceB.Get();
  Robot::Intake.lastStates[2] = Robot::Intake.beltA.Get();
	Robot::Intake.lastStates[3] = Robot::Intake.beltB.Get();
  Robot::Intake.lastStates[4] = Robot::Intake.beltC.Get();
	Robot::Intake.lastStates[5] = Robot::Intake.beltD.Get();

  /*if(!Robot::oi.OperatorController->GetRawButton(MANUAL_OPERATOR_OVERRIDE_BUTTON)){
    if(Robot::Intake.manualEnabled && Robot::Intake.indexEnabled){
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, -0.5);
    }else{
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
    }
    
    
  }else{
    // operator is overriding the mechanism
    if(Robot::oi.OperatorController->GetRawButton(OPERATOR_INTAKE_FEED_ROBOT_BUTTON)){
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, -0.5);
    }else if(Robot::oi.OperatorController->GetRawButton(OPERATOR_INTAKE_REJECT_BUTTON)){
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, 0.5);
    }else{
      Robot::Intake.intakeMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
    }
  }*/
}

// Make this return true when this Command no longer needs to run execute()
bool DefaultIntakeCommand::IsFinished() { return false; }

// Called once after isFinished returns true
void DefaultIntakeCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DefaultIntakeCommand::Interrupted() {}
