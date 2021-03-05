/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Commands.h"

#include "Robot.h"
#include <cmath>
#include "PIDConstants.h"

trackCommand::trackCommand() {
  // Use Requires() here to declare subsystem dependencies
  //Requires(&Robot::Indexer);
  Requires(&Robot::Drive);
  Requires(&Robot::Shooter);
}

// Called just before this Command runs the first time
void trackCommand::Initialize() {
  //frc::SmartDashboard::PutNumber("Track P", PC);
  //frc::SmartDashboard::PutNumber("Track I", IC);
  //frc::SmartDashboard::PutNumber("Track D", DC);
}

// Called repeatedly when this Command is scheduled to run
void trackCommand::Execute() {
  float driverJoyY = std::pow(Robot::oi.DriverController->GetRawAxis(DRIVE_CONTROLLER_DRIVE_AXIS_Y), DRIVE_SENSITIVITY_FACTOR);
  float lastTx = tx;
  tx = Robot::table->GetEntry("tx").GetDouble(0.0);

  //frc::SmartDashboard::PutNumber("txDebug", tx);
  //frc::SmartDashboard::PutNumber("tyDebug", ty);
  //PC = frc::SmartDashboard::GetNumber("Track P", 0.0);
  //IC = frc::SmartDashboard::GetNumber("Track I", 0.0);
  //DC = frc::SmartDashboard::GetNumber("Track D", 0.0);
  
  rP = tx;
  rI = .9 * tx + .1 * rI;
  rD = lastTx - tx;
  float rPID = TRACK_HORIZONTAL_P * rP + TRACK_HORIZONTAL_I * rI + TRACK_HORIZONTAL_D * rD;
  Robot::Drive.driveControl.ArcadeDrive(driverJoyY, -rPID);

  ty = Robot::table->GetEntry("ty").GetDouble(0.0);
  
  double angles[3];
  Robot::Shooter.pigeon.GetYawPitchRoll(angles);

  frc::SmartDashboard::PutNumber("Pigeon Pitch", angles[1]);

  float RAD_SCALE = 3.1415 / 180;

  float pitch = RAD_SCALE * angles[1];

  float lx = std::cos(0.628319 + pitch) * 20.4022057;
  float ly = std::sin(0.628319 + pitch) * 20.4022057 + 17;

  float d = (90.5 - ly) / (std::tan(pitch + (ty * RAD_SCALE))) + lx;

  frc::SmartDashboard::PutNumber("Distance Estimator", d);

  

  Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::Position, GetBallisticAngle(d));
  float speed = GetBallisticSpeed(d);

  frc::SmartDashboard::PutNumber("RPM target", speed);

  Robot::Shooter.left.Set(motorcontrol::ControlMode::Velocity, speed);
  Robot::Shooter.right.Set(motorcontrol::ControlMode::Velocity, speed);
  

}

float trackCommand::GetBallisticSpeed(float dist){
  float A = .2;
  float B = 12;
  return 12288 + (8192) * 0.5 * (std::tanh(((dist / 12) - B) * A) + 1);
}

float trackCommand::GetBallisticAngle(float dist){
  float angleFactor = - 150 / 6.5;
  float A = .13;
  float B = 0;
  frc::SmartDashboard::PutNumber("Target angle", (80 - 55 * std::tanh((dist - B) * A)));
  return angleFactor * (67 - 38 * std::tanh(((dist / 12) - B) * A));
}

// Make this return true when this Command no longer needs to run execute()
bool trackCommand::IsFinished() { return !Robot::oi.DriverController->GetRawButton(DRIVE_TRACK_BUTTON); }

// Called once after isFinished returns true
void trackCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void trackCommand::Interrupted() {}
