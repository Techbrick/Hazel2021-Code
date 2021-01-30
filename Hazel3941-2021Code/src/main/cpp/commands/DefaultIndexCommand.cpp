/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Commands.h"

#include "Robot.h"

DefaultIndexCommand::DefaultIndexCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Indexer);
}

// Called just before this Command runs the first time
void DefaultIndexCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultIndexCommand::Execute() {
	//frc::SmartDashboard::PutNumberArray("ballArray", Robot::Indexer.balls);
	frc::SmartDashboard::PutNumber("ballArraySize", Robot::Indexer.balls.size());
	if(!Robot::oi.OperatorController->GetRawButton(MANUAL_OPERATOR_OVERRIDE_BUTTON)){
		if(Robot::Indexer.drivenManually == true){
			Robot::Indexer.beltOn = false;
			Robot::Indexer.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
		}
		if(Robot::Intake.manualEnabled && ((!Robot::Indexer.distanceA.Get() && Robot::Indexer.lastStates[0]) || (!Robot::Indexer.distanceB.Get() && Robot::Indexer.lastStates[1]))){
			Robot::Indexer.balls.push_back(Ball(-100));
		}
		Robot::Indexer.lastStates[0] = Robot::Indexer.distanceA.Get();
		Robot::Indexer.lastStates[1] = Robot::Indexer.distanceB.Get();
		frc::SmartDashboard::PutBoolean("beltOn", Robot::Indexer.beltOn);
		if(Robot::Indexer.balls.size() > 0){
			frc::SmartDashboard::PutNumber("lastBallPos", Robot::Indexer.balls.at(Robot::Indexer.balls.size() - 1).x);
		}
		if(Robot::Indexer.balls.size() != 0 && Robot::Indexer.balls.at(Robot::Indexer.balls.size() - 1).x < 0){
			Robot::Indexer.beltOn = true;
			if(Robot::Indexer.lastStates[2] != Robot::Indexer.beltA.Get()){
				if(Robot::Indexer.lastStates[2]){
					Robot::Indexer.lastStates[2] = false;
					Robot::Indexer.balls.at(0).x = -90;
				}else{
					Robot::Indexer.lastStates[2] = true;
					Robot::Indexer.balls.at(0).x = -80;
				}
			}
			if(Robot::Indexer.lastStates[3] != Robot::Indexer.beltB.Get()){
				if(Robot::Indexer.lastStates[3]){
					Robot::Indexer.lastStates[3] = false;
					Robot::Indexer.balls.at(0).x = -70;
				}else{
					Robot::Indexer.lastStates[3] = true;
					Robot::Indexer.balls.at(0).x = -60;
				}
			}
			if(Robot::Indexer.lastStates[4] != Robot::Indexer.beltC.Get()){
				if(Robot::Indexer.lastStates[4]){
					Robot::Indexer.lastStates[4] = false;
					Robot::Indexer.balls.at(0).x = -30;
					Robot::Indexer.indexWheelOn = true;
				}else{
					Robot::Indexer.lastStates[4] = true;
					Robot::Indexer.balls.at(0).x = -20;
				}
			}
			if(Robot::Indexer.lastStates[5] != Robot::Indexer.beltD.Get()){
				if(Robot::Indexer.lastStates[5]){
					Robot::Indexer.lastStates[5] = false;
					Robot::Indexer.balls.at(0).x = -10;
				}else{
					Robot::Indexer.lastStates[5] = true;
					Robot::Indexer.balls.at(0).x = 0;
					Robot::Indexer.indexWheelOn = false;
				}
			}
		}else{
			Robot::Indexer.beltOn = false;
		}
		frc::SmartDashboard::PutBoolean("wheelOn", Robot::Indexer.indexWheelOn);
		if(Robot::Intake.extended && Robot::Indexer.beltOn && Robot::Intake.manualEnabled){
			Robot::Indexer.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, 1.0);
		}else{
			Robot::Indexer.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
		}
		if(Robot::Indexer.indexWheelOn && Robot::Intake.manualEnabled){
			Robot::Indexer.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 1.0);
		}else{
			Robot::Indexer.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
		}
	}else{
		Robot::Indexer.drivenManually = true;
		if(Robot::oi.OperatorController->GetRawButton(OPERATOR_INDEXER_FEED_FORWARD_BUTTON)){
			Robot::Indexer.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, 1);
			Robot::Indexer.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 1);
		}else if(Robot::oi.OperatorController->GetRawButton(OPERATOR_INDEXER_FEED_REVERSE_BUTTON)){
			Robot::Indexer.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, -1);
			Robot::Indexer.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, -1);
		}else{
			Robot::Indexer.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
			Robot::Indexer.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
		}
  	}
}

// Make this return true when this Command no longer needs to run execute()
bool DefaultIndexCommand::IsFinished() { return false; }

// Called once after isFinished returns true
void DefaultIndexCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DefaultIndexCommand::Interrupted() {}
