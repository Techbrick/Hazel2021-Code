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
  float cap = .45;
  if(abs(rPID) > cap){
    rPID = (float)((rPID > 0) - (rPID < 0)) * cap;
  }
  Robot::Drive.driveControl.ArcadeDrive(driverJoyY, -rPID, false);

  ty = Robot::table->GetEntry("ty").GetDouble(0.0);
  
  double angles[3];
  Robot::Shooter.pigeon.GetYawPitchRoll(angles);

  frc::SmartDashboard::PutNumber("Pigeon Pitch", angles[1]);

  float RAD_SCALE = 3.1415 / 180;

  float pitch = RAD_SCALE * angles[1];

  // height of pivot 18.4 in
  // angle offset at level of limelight 0.568524548
  // dist from pivot to limelight 21.36001

  float lx = std::cos(0.568524548 + pitch) * 21.36001;
  float ly = std::sin(0.568524548 + pitch) * 21.36001 + 18.25;

  float d = (90.5 - ly) / (std::tan(pitch + (ty * RAD_SCALE))) + lx;

  frc::SmartDashboard::PutNumber("Distance Estimator", d);

  

  Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::Position, GetBallisticAngle(d));
  float speed = GetBallisticSpeed(d);

  frc::SmartDashboard::PutNumber("RPM target", speed);

  Robot::Shooter.left.Set(motorcontrol::ControlMode::Velocity, speed-0);
  Robot::Shooter.right.Set(motorcontrol::ControlMode::Velocity, speed-0);
  

}

float trackCommand::GetBallisticSpeed(float dist){
  float A = .2;
  float B = 12;
  return 12288 + (8192) * 0.5 * (std::tanh(((dist / 12) - B) * A) + 1);
}

float trackCommand::GetBallisticAngle(float dist){
  float angleFactor = - 150 / 6.5;
  float A = .12;
  float B = -.6;
  // 90 - 61 * tanh[(x/12 - (-0.6)) * 0.12]
  frc::SmartDashboard::PutNumber("Target angle", (90 - 61 * std::tanh(((dist / 12) - B) * A)));
  return angleFactor * (90 - 61 * std::tanh(((dist / 12) - B) * A));
}

// Make this return true when this Command no longer needs to run execute()
bool trackCommand::IsFinished() { return !Robot::oi.DriverController->GetRawButton(DRIVE_TRACK_BUTTON); }

// Called once after isFinished returns true
void trackCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void trackCommand::Interrupted() {}
