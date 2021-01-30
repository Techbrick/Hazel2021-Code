/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Commands.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "Robot.h"
#include "RobotMap.h"

DefaultDriveCommand::DefaultDriveCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Drive);
}

// Called just before this Command runs the first time
void DefaultDriveCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultDriveCommand::Execute() {
    frc::SmartDashboard::PutNumber("Axis X", Robot::oi.DriverController->GetRawAxis(DRIVE_CONTROLLER_DRIVE_AXIS_X));
    double joyX = -Robot::oi.DriverController->GetRawAxis(DRIVE_CONTROLLER_DRIVE_AXIS_X);
    double joyY = Robot::oi.DriverController->GetRawAxis(DRIVE_CONTROLLER_DRIVE_AXIS_Y);
    //joyY = 0;
    //joyZ = 0;
    /*
    if(abs(joyX) <= .05){
        Robot::Drive.RightController.Set(motorcontrol::ControlMode::PercentOutput, joyY);
        Robot::Drive.LeftController.Set(motorcontrol::ControlMode::PercentOutput, joyY);
    }else{
        Robot::Drive.RightController.Set(motorcontrol::ControlMode::PercentOutput, joyX / 2);
        Robot::Drive.LeftController.Set(motorcontrol::ControlMode::PercentOutput, -joyX / 2);
    }
    if(abs(joyX) <= 0.1){
        joyX = 0;
    }
    if(abs(joyY) <= 0.1){
        joyY = 0;
    }
    */
    /*if(abs(joyX) < 0.015){
        joyX = 0;
    }
    if(abs(joyY) < 0.015){
        joyY = 0;
    }*/
    Robot::Drive.driveControl.ArcadeDrive(joyY, joyX, false);


    if(!(Robot::oi.DriverController->GetRawAxis(DRIVE_SHIFT_AXIS) > 0.75)){
        Robot::Drive.ShifterSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }else{
        Robot::Drive.ShifterSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
}

// Make this return true when this Command no longer needs to run execute()
bool DefaultDriveCommand::IsFinished() { return false; }

// Called once after isFinished returns true
void DefaultDriveCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DefaultDriveCommand::Interrupted() {}