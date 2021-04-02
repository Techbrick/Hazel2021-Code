/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Commands.h"

#include "Robot.h"

DefaultShooterCommand::DefaultShooterCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Shooter);
}

// Called just before this Command runs the first time
void DefaultShooterCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultShooterCommand::Execute() {
  bool lowerlimstatus = !Robot::Shooter.lowerLim.Get();

  double angles[3];
  Robot::Shooter.pigeon.GetYawPitchRoll(angles);

  //frc::SmartDashboard::PutNumber("Pigeon Yaw", angles[0]);
  frc::SmartDashboard::PutNumber("Pigeon Pitch", angles[1]);
  //frc::SmartDashboard::PutNumber("Pigeon Roll", angles[2]);

  bool axis_locked = Robot::oi.OperatorController->GetRawButton(OPERATOR_LOCKED_AXIS_OVERRIDE);
  bool operator_override = Robot::oi.OperatorController->GetRawButton(MANUAL_OPERATOR_OVERRIDE_BUTTON) || axis_locked;

  if(!operator_override){
    if(Robot::Shooter.drivenManually == true){
      Robot::Shooter.drivenManually = false;
      Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
      Robot::Shooter.left.Set(motorcontrol::ControlMode::PercentOutput,0);
    }
    if(/*!Robot::Shooter.lowerLim.Get() && */Robot::oi.DriverController->GetRawButton(DRIVE_CONTROLLER_ROLL_SHOOTER_BACKWARD)){
      Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::PercentOutput, -0.2);
    }else if( lowerlimstatus && Robot::oi.DriverController->GetRawButton(DRIVE_CONTROLLER_ROLL_SHOOTER_FORWARD)){
      Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::PercentOutput, 0.2);
    }else{
      Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
    }
    /*if(Robot::oi.DriverController->GetRawButton(DRIVE_ARM_BUTTON)){
      Robot::Shooter.left.Set(motorcontrol::ControlMode::PercentOutput, 1.0);
      Robot::Shooter.right.Set(motorcontrol::ControlMode::PercentOutput, 1.0);
    }else{
      Robot::Shooter.left.Set(motorcontrol::ControlMode::PercentOutput, 0.0);
      Robot::Shooter.right.Set(motorcontrol::ControlMode::PercentOutput, 0.0);
    }*/
    
  } else {
    // operator is taking control manually
    Robot::Shooter.drivenManually = true;
    if(!axis_locked){
      double armmove = Robot::oi.OperatorController->GetRawAxis(OPERATOR_JOYSTICK_FORWARDREVERSE_AXIS_ID);
      if(OPERATOR_JOYSTICK_FORWARDREVERSE_FLIP){
        armmove = -armmove;
      }
      if(lowerlimstatus && armmove > 0){
        Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::PercentOutput, armmove);
      } else if( armmove < 0){
        Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::PercentOutput, armmove);
      } else {
        Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
      }
    }else{
      Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::PercentOutput, 0);
    }
    
    if(Robot::oi.OperatorController->GetRawButton(OPERATOR_SHOOTER_SPIN_ENABLE_BUTTON)){
        double inputspeed = 0.0;
        if(OPERATOR_SHOOTER_SPEED_AXIS_FLIP){
          // shift value higher, remap to 0-1, flip
          inputspeed = (Robot::oi.OperatorController->GetRawAxis(OPERATOR_SHOOTER_SPEED_AXIS_ID) + 1) / 2;
        } else {
          // shift value higher, remap to 0-1
          
          inputspeed = (-Robot::oi.OperatorController->GetRawAxis(OPERATOR_SHOOTER_SPEED_AXIS_ID) + 1) / 2;
        }
        double maxRPM = 2000;
        double minRPM = 1200;
        double rpm = minRPM + (maxRPM - minRPM) * abs(inputspeed);
        rpm = rpm * 3 * 2048 / 600;
        Robot::Shooter.left.Set(motorcontrol::ControlMode::Velocity, rpm);
        Robot::Shooter.right.Set(motorcontrol::ControlMode::Velocity, rpm);
        //Robot::Shooter.left.Set(motorcontrol::ControlMode::PercentOutput, -inputspeed);
        frc::SmartDashboard::PutNumber("shooter power", rpm);
      } else {
        Robot::Shooter.left.Set(motorcontrol::ControlMode::PercentOutput, 0);
        Robot::Shooter.right.Set(motorcontrol::ControlMode::PercentOutput, 0);
      }
  }
}

// Make this return true when this Command no longer needs to run execute()
bool DefaultShooterCommand::IsFinished() { return false; }

// Called once after isFinished returns true
void DefaultShooterCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DefaultShooterCommand::Interrupted() {}
