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
    // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {
        // Caitlyn's U turn
        //frc::Translation2d(1.0_m, 0.0_m),
        //frc::Translation2d(1.5_m, 0.5_m),
        //frc::Translation2d(1.0_m, 1_m)
        // Rich's Slalom
        //frc::Translation2d(0_m, 0_m),
        frc::Translation2d(0.457317073170732_m, 0_m),
        frc::Translation2d(0.76219512195122_m, 0.152439024390244_m),
        frc::Translation2d(1.06707317073171_m, 0.518292682926829_m),
        frc::Translation2d(1.21951219512195_m, 0.76219512195122_m),
        frc::Translation2d(1.3719512195122_m, 1.00609756097561_m),
        frc::Translation2d(1.67682926829268_m, 1.3719512195122_m),
        frc::Translation2d(1.98170731707317_m, 1.52439024390244_m),
        frc::Translation2d(2.59146341463415_m, 1.67682926829268_m),
        frc::Translation2d(3.50609756097561_m, 1.67682926829268_m),
        frc::Translation2d(4.42073170731707_m, 1.67682926829268_m),
        frc::Translation2d(5.03048780487805_m, 1.52439024390244_m),
        frc::Translation2d(5.33536585365854_m, 1.3719512195122_m),
        frc::Translation2d(5.64024390243902_m, 1.00609756097561_m),
        frc::Translation2d(5.79268292682927_m, 0.76219512195122_m),
        frc::Translation2d(5.94512195121951_m, 0.518292682926829_m),
        frc::Translation2d(6.25_m, 0.152439024390244_m),
        frc::Translation2d(6.61585365853658_m, 0_m),
        frc::Translation2d(7.01219512195122_m, 0.152439024390244_m),
        frc::Translation2d(7.31707317073171_m, 0.457317073170732_m),
        frc::Translation2d(7.46951219512195_m, 0.76219512195122_m),
        frc::Translation2d(7.31707317073171_m, 1.06707317073171_m),
        frc::Translation2d(7.01219512195122_m, 1.3719512195122_m),
        frc::Translation2d(6.61585365853658_m, 1.52439024390244_m),
        frc::Translation2d(6.25_m, 1.3719512195122_m),
        frc::Translation2d(5.94512195121951_m, 1.00609756097561_m),
        frc::Translation2d(5.79268292682927_m, 0.76219512195122_m),
        frc::Translation2d(5.64024390243902_m, 0.518292682926829_m),
        frc::Translation2d(5.33536585365854_m, 0.152439024390244_m),
        frc::Translation2d(5.03048780487805_m, 0_m),
        frc::Translation2d(4.42073170731707_m, -0.0914634146341463_m),
        frc::Translation2d(3.50609756097561_m, -0.0914634146341463_m),
        frc::Translation2d(2.59146341463415_m, -0.0914634146341463_m),
        frc::Translation2d(1.98170731707317_m, 0_m),
        frc::Translation2d(1.67682926829268_m, 0.152439024390244_m),
        frc::Translation2d(1.3719512195122_m, 0.518292682926829_m),
        frc::Translation2d(1.21951219512195_m, 0.76219512195122_m),
        frc::Translation2d(1.06707317073171_m, 1.00609756097561_m),
        frc::Translation2d(0.76219512195122_m, 1.3719512195122_m),
        frc::Translation2d(0.457317073170732_m, 1.52439024390244_m),
        //frc::Translation2d(0_m, 1.52439024390244_m),
        //frc::Translation2d(-0.152439024390244_m, 1.52439024390244_m),

      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(0_m, 1.5_m, frc::Rotation2d(180_deg)),
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
