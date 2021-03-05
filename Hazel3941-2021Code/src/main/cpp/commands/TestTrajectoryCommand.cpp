/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/AutoCommands.h"

#include "Robot.h"

TestTrajectoryCommand::TestTrajectoryCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Drive);
  trajectoryConfig.SetKinematics(kDriveKinematics);
  trajectoryConfig.AddConstraint(autoVoltageConstraint);

  trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      trajectoryConfig
  );
}

// Called just before this Command runs the first time
void TestTrajectoryCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void TestTrajectoryCommand::Execute() {}

// Make this return true when this Command no longer needs to run execute()
bool TestTrajectoryCommand::IsFinished() { return false; }

// Called once after isFinished returns true
void TestTrajectoryCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TestTrajectoryCommand::Interrupted() {}
