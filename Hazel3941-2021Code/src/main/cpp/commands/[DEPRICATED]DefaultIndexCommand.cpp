/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/*


#include "commands/Commands.h"

#include "Robot.h"

DefaultIndexCommand::DefaultIndexCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Intake);
}

// Called just before this Command runs the first time
void DefaultIndexCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultIndexCommand::Execute() {
	//frc::SmartDashboard::PutNumberArray("ballArray", Robot::Intake.balls);
	//frc::SmartDashboard::PutNumber("ballArraySize", Robot::Intake.balls.size());
	if(!Robot::oi.OperatorController->GetRawButton(MANUAL_OPERATOR_OVERRIDE_BUTTON)){
		if(Robot::Intake.drivenManually == true){
			Robot::Intake.beltOn = false;
			Robot::Intake.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
		}
		if(Robot::Intake.manualEnabled && ((!Robot::Intake.distanceA.Get() && Robot::Intake.lastStates[0]) || (!Robot::Intake.distanceB.Get() && Robot::Intake.lastStates[1]))){
			Robot::Intake.balls.push_back(Ball(-100));
		}
		Robot::Intake.lastStates[0] = Robot::Intake.distanceA.Get();
		Robot::Intake.lastStates[1] = Robot::Intake.distanceB.Get();
		frc::SmartDashboard::PutBoolean("beltOn", Robot::Intake.beltOn);
		if(Robot::Intake.balls.size() > 0){
			frc::SmartDashboard::PutNumber("lastBallPos", Robot::Intake.balls.at(Robot::Intake.balls.size() - 1).x);
		}
		if(Robot::Intake.balls.size() != 0 && Robot::Intake.balls.at(Robot::Intake.balls.size() - 1).x < 0){
			Robot::Intake.beltOn = true;
			if(Robot::Intake.lastStates[2] != Robot::Intake.beltA.Get()){
				if(Robot::Intake.lastStates[2]){
					Robot::Intake.lastStates[2] = false;
					Robot::Intake.balls.at(0).x = -90;
				}else{
					Robot::Intake.lastStates[2] = true;
					Robot::Intake.balls.at(0).x = -80;
				}
			}
			if(Robot::Intake.lastStates[3] != Robot::Intake.beltB.Get()){
				if(Robot::Intake.lastStates[3]){
					Robot::Intake.lastStates[3] = false;
					Robot::Intake.balls.at(0).x = -70;
				}else{
					Robot::Intake.lastStates[3] = true;
					Robot::Intake.balls.at(0).x = -60;
				}
			}
			if(Robot::Intake.lastStates[4] != Robot::Intake.beltC.Get()){
				if(Robot::Intake.lastStates[4]){
					Robot::Intake.lastStates[4] = false;
					Robot::Intake.balls.at(0).x = -30;
					Robot::Intake.indexWheelOn = true;
				}else{
					Robot::Intake.lastStates[4] = true;
					Robot::Intake.balls.at(0).x = -20;
				}
			}
			if(Robot::Intake.lastStates[5] != Robot::Intake.beltD.Get()){
				if(Robot::Intake.lastStates[5]){
					Robot::Intake.lastStates[5] = false;
					Robot::Intake.balls.at(0).x = -10;
				}else{
					Robot::Intake.lastStates[5] = true;
					Robot::Intake.balls.at(0).x = 0;
					Robot::Intake.indexWheelOn = false;
				}
			}
		}else{
			Robot::Intake.beltOn = false;
		}
		//frc::SmartDashboard::PutBoolean("wheelOn", Robot::Intake.indexWheelOn);
		if(Robot::Intake.extended && Robot::Intake.beltOn && Robot::Intake.manualEnabled){
			Robot::Intake.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, 1.0);
		}else{
			Robot::Intake.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
		}
		if(Robot::Intake.indexWheelOn && Robot::Intake.manualEnabled){
			Robot::Intake.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 1.0);
		}else{
			Robot::Intake.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
		}
	}else{
		Robot::Intake.drivenManually = true;
		if(Robot::oi.OperatorController->GetRawButton(OPERATOR_Intake_FEED_FORWARD_BUTTON)){
			Robot::Intake.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, 1);
			Robot::Intake.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 1);
		}else if(Robot::oi.OperatorController->GetRawButton(OPERATOR_Intake_FEED_REVERSE_BUTTON)){
			Robot::Intake.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, -1);
			Robot::Intake.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, -1);
		}else{
			Robot::Intake.beltMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
			Robot::Intake.indexWheelMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
		}
  	}
}

// Make this return true when this Command no longer needs to run execute()
bool DefaultIndexCommand::IsFinished() { return false; }

// Called once after isFinished returns true
void DefaultIndexCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DefaultIndexCommand::Interrupted() {}*/
