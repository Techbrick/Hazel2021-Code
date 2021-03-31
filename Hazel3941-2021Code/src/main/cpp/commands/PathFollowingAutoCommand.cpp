/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoCommands.h"

#include "Robot.h"

PathFollowingAutoCommand::PathFollowingAutoCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Drive);
}

// Called just before this Command runs the first time
void PathFollowingAutoCommand::Initialize() {
  timer = 100;
  Robot::Drive.navx.ZeroYaw();
  Robot::Drive.ResetOdometryPose();

  trajectoryConfig.SetKinematics(kDriveKinematics);
  trajectoryConfig.AddConstraint(autoVoltageConstraint);

  trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d(0_m, 0_m, 0_deg),
    {
      frc::Translation2d(0.5_m, 0_m),
      frc::Translation2d(1_m, 0.5_m),
    },
    frc::Pose2d(1_m, 1_m, 90_deg),
    trajectoryConfig
  );

  ramseteCommand = new frc2::RamseteCommand(
    trajectory,
    [this]() { return Robot::Drive.odometry.GetPose(); },
    frc::RamseteController(kRamseteB, kRamseteZeta),
    frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
    kDriveKinematics,
    [this]() { return Robot::Drive.GetWheelSpeeds(); },
    frc2::PIDController(kPDriveVel, 0, 0),
    frc2::PIDController(kPDriveVel, 0, 0),
    [this](units::volt_t left, units::volt_t right) { 
      Robot::Drive.TankDriveVolts(left, -right); 
      frc::SmartDashboard::PutNumber("CommandedLeftVolts", left.value());
      frc::SmartDashboard::PutNumber("CommandedRightVolts", -right.value()); }
  );

  ramseteCommand->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void PathFollowingAutoCommand::Execute() {
  ramseteCommand->Execute();
  frc::SmartDashboard::PutNumber("LeftWheelSpeed", Robot::Drive.GetWheelSpeeds().left.value());
  frc::SmartDashboard::PutNumber("RightWheelSpeed", Robot::Drive.GetWheelSpeeds().right.value());

}

// Make this return true when this Command no longer needs to run execute()
bool PathFollowingAutoCommand::IsFinished() { 
  return ramseteCommand->IsFinished();
}

// Called once after isFinished returns true  
void PathFollowingAutoCommand::End() {
  ramseteCommand->End(false);
  Robot::Drive.TankDriveVolts(0_V, 0_V);
  free(ramseteCommand);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PathFollowingAutoCommand::Interrupted() {
  ramseteCommand->End(true);
  Robot::Drive.TankDriveVolts(0_V, 0_V);
  free(ramseteCommand);
}
