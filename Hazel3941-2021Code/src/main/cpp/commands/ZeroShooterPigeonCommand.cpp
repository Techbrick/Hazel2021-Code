/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Commands.h"

#include "Robot.h"

ZeroShooterPigeonCommand::ZeroShooterPigeonCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Shooter);
}

// Called just before this Command runs the first time
void ZeroShooterPigeonCommand::Initialize() {
  //frc::SmartDashboard::PutBoolean("Is zeroing?", true);
  Robot::Shooter.isShooterZeroed = false;
}

// Called repeatedly when this Command is scheduled to run
void ZeroShooterPigeonCommand::Execute() {
  bool hasHitUpperLimit = Robot::Shooter.armMotor.IsRevLimitSwitchClosed();
  if (hasHitUpperLimit) {
    // If we hit the limit, stop..
    Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::PercentOutput, 0.0);

    // ...and zero...
    Robot::Shooter.pigeon.SetYaw(SHOOTER_TOP_LIMIT_ANGLE);
    Robot::Shooter.isShooterZeroed = true;
  }else{
    // ...otherwise keep going...
    Robot::Shooter.armMotor.Set(motorcontrol::ControlMode::PercentOutput, -0.2);
  }
}

// Make this return true when this Command no longer needs to run execute()
bool ZeroShooterPigeonCommand::IsFinished() {
  //frc::SmartDashboard::PutBoolean("Is zeroing?", Robot::Shooter.isShooterZeroed);
  return Robot::Shooter.isShooterZeroed;
}

// Called once after isFinished returns true
void ZeroShooterPigeonCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ZeroShooterPigeonCommand::Interrupted() {}
