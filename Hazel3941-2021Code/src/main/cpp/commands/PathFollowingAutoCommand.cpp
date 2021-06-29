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
  Requires(&Robot::TrajectoryTiming);
}

// Called just before this Command runs the first time
void PathFollowingAutoCommand::Initialize() {
  path_index = 0;
  Robot::Drive.navx.ZeroYaw();
  Robot::Drive.ResetOdometryPose();
  frc::Trajectory firstTrajectory = Robot::TrajectoryTiming.pathChooser.GetSelected()[path_index];

  ramseteCommand = new frc2::RamseteCommand(
    firstTrajectory,
    [this]() { return Robot::Drive.odometry.GetPose(); },
    frc::RamseteController(kRamseteB, kRamseteZeta),
    frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
    Robot::TrajectoryTiming.kDriveKinematics,
    [this]() { return Robot::Drive.GetWheelSpeeds(); },
    frc2::PIDController(kPDriveVel, 0, 0),
    frc2::PIDController(kPDriveVel, 0, 0),
    [this](units::volt_t left, units::volt_t right) { 
      Robot::Drive.TankDriveVolts(left, -right);
    }
  );

  ramseteCommand->Initialize();
}

// Called repeatedly when this Command is scheduled to run
void PathFollowingAutoCommand::Execute() {
  ramseteCommand->Execute();

  // Check if the current command is done
  if (ramseteCommand->IsFinished()) {
    // End normally
    ramseteCommand->End(false);
    Robot::Drive.TankDriveVolts(0_V, 0_V);
    free(ramseteCommand);

    // Increment path index
    path_index++;

    if (path_index <= Robot::TrajectoryTiming.pathChooser.GetSelected().size()) {
      frc::Trajectory nextTrajectory = Robot::TrajectoryTiming.pathChooser.GetSelected()[path_index];

      ramseteCommand = new frc2::RamseteCommand(
        nextTrajectory,
        [this]() { return Robot::Drive.odometry.GetPose(); },
        frc::RamseteController(kRamseteB, kRamseteZeta),
        frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
        Robot::TrajectoryTiming.kDriveKinematics,
        [this]() { return Robot::Drive.GetWheelSpeeds(); },
        frc2::PIDController(kPDriveVel, 0, 0),
        frc2::PIDController(kPDriveVel, 0, 0),
        [this](units::volt_t left, units::volt_t right) { 
          Robot::Drive.TankDriveVolts(left, -right);
        }
      );

      ramseteCommand->Initialize();
    }
  }
}

// Make this return true when this Command no longer needs to run execute()
bool PathFollowingAutoCommand::IsFinished() { 
  return path_index > Robot::TrajectoryTiming.pathChooser.GetSelected().size();
}

// Called once after isFinished returns true  
void PathFollowingAutoCommand::End() {
  
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void PathFollowingAutoCommand::Interrupted() {
  ramseteCommand->End(true);
  Robot::Drive.TankDriveVolts(0_V, 0_V);
  free(ramseteCommand);
}
