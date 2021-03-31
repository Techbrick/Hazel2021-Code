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

  /*trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      {
        // Slalom
        //frc::Translation2d(0_m, 0_m),
        frc::Translation2d(0.457317073170732_m, 0_m),
        frc::Translation2d(0.76219512195122_m, 0.152439024390244_m),
        frc::Translation2d(1.06707317073171_m, 0.518292682926829_m),
        frc::Translation2d(1.21951219512195_m, 0.76219512195122_m), // *
        frc::Translation2d(1.3719512195122_m, 1.00609756097561_m),
        frc::Translation2d(1.67682926829268_m, 1.3719512195122_m),
        frc::Translation2d(1.98170731707317_m, 1.52439024390244_m),
        frc::Translation2d(2.59146341463415_m, 1.67682926829268_m),
        frc::Translation2d(3.50609756097561_m, 1.67682926829268_m),
        frc::Translation2d(4.42073170731707_m, 1.67682926829268_m),
        frc::Translation2d(5.03048780487805_m, 1.52439024390244_m),
        frc::Translation2d(5.33536585365854_m, 1.3719512195122_m),
        frc::Translation2d(5.64024390243902_m, 1.00609756097561_m),
        frc::Translation2d(5.79268292682927_m, 0.76219512195122_m), // *
        frc::Translation2d(5.94512195121951_m, 0.518292682926829_m),
        frc::Translation2d(6.25_m, 0.152439024390244_m),
        frc::Translation2d(6.61585365853658_m, 0_m),
        frc::Translation2d(7.01219512195122_m, 0.152439024390244_m),
        frc::Translation2d(7.31707317073171_m, 0.457317073170732_m),
        frc::Translation2d(7.46951219512195_m, 0.76219512195122_m), // *
        frc::Translation2d(7.31707317073171_m, 1.06707317073171_m),
        frc::Translation2d(7.01219512195122_m, 1.3719512195122_m),
        frc::Translation2d(6.61585365853658_m, 1.52439024390244_m),
        frc::Translation2d(6.25_m, 1.3719512195122_m),
        frc::Translation2d(5.94512195121951_m, 1.00609756097561_m),
        frc::Translation2d(5.79268292682927_m, 0.76219512195122_m), // *
        frc::Translation2d(5.64024390243902_m, 0.518292682926829_m),
        frc::Translation2d(5.33536585365854_m, 0.152439024390244_m),
        frc::Translation2d(5.03048780487805_m, 0_m),
        frc::Translation2d(4.42073170731707_m, -0.0914634146341463_m),
        frc::Translation2d(3.50609756097561_m, -0.0914634146341463_m),
        frc::Translation2d(2.59146341463415_m, -0.0914634146341463_m),
        frc::Translation2d(1.98170731707317_m, 0_m),
        frc::Translation2d(1.67682926829268_m, 0.152439024390244_m),
        frc::Translation2d(1.3719512195122_m, 0.518292682926829_m),
        frc::Translation2d(1.21951219512195_m, 0.76219512195122_m), // *
        frc::Translation2d(1.06707317073171_m, 1.00609756097561_m),
        frc::Translation2d(0.76219512195122_m, 1.3719512195122_m),
        frc::Translation2d(0.457317073170732_m, 1.52439024390244_m)
        //frc::Translation2d(0_m, 1.52439024390244_m),
        //frc::Translation2d(-0.152439024390244_m, 1.52439024390244_m),

      },
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(0_m, 1.5_m, frc::Rotation2d(180_deg)),
      trajectoryConfig
  );*/

  /*trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    // Start at the origin facing the +X direction
      // Pass through these two interior waypoints, making an 's' curve path
      {
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        // Caitlyn's U turn
        frc::Pose2d(1.0_m, 0.0_m, 0_deg),
        //frc::Pose2d(1.353_m, 0.353_m, 45_deg),
        frc::Pose2d(1.5_m, 0.5_m, 90_deg),
        //frc::Pose2d(1.353_m, .707_m, 135_deg),
        frc::Pose2d(1_m, 1_m, 180_deg),
        frc::Pose2d(0_m, 1_m, frc::Rotation2d(180_deg))
        // Rich's Slalom
      },
      // End 3 meters straight ahead of where we started, facing forward
      trajectoryConfig
  );*/

  trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    // Start at the origin facing the +X direction
    // Compressed in Y Barrel Run
      frc::Pose2d(0_m, 0_m, 0_deg),
      {
frc::Translation2d(3.61450916527799_m, -0.262744281169519_m),
frc::Translation2d(3.85594985608241_m, -1.03677473110135_m),
frc::Translation2d(3.3446636873201_m, -1.5_m),
frc::Translation2d(2.39310331767914_m, -1.32792379942433_m),
frc::Translation2d(2.15166262687472_m, -0.781131646720194_m),
frc::Translation2d(2.24397818512347_m, -0.369262232995001_m),
frc::Translation2d(2.74106196030904_m, -0.085214361460384_m),
frc::Translation2d(3.23814573549462_m, 0.0923155582487502_m),
frc::Translation2d(4.01927738221482_m, 0.156226329344039_m),
frc::Translation2d(4.83591501287684_m, -0.336672473867596_m),
frc::Translation2d(5.34720118163915_m, -0.393482048174519_m),
frc::Translation2d(5.95790410543857_m, -0.0810293894864412_m),
frc::Translation2d(6.01471367974549_m, 0.444459172852599_m),
frc::Translation2d(5.84428495682472_m, 0.764013028329041_m),
frc::Translation2d(5.42531434631116_m, 0.998352522345099_m),
frc::Translation2d(4.78562339039539_m, 0.941542948038176_m),
frc::Translation2d(4.25_m, 0.352143614603848_m),
frc::Translation2d(4.5_m, -0.2_m),
frc::Translation2d(5.92239812149674_m, -0.916054385699136_m),
frc::Translation2d(6.34846992879867_m, -1.19300106044539_m),
frc::Translation2d(6.81004772004242_m, -1.41313816088471_m),
frc::Translation2d(7.4491554309953_m, -1.46284653840327_m),
frc::Translation2d(7.79001287683684_m, -1.07228071504318_m),
frc::Translation2d(7.73320330252992_m, -0.596500530222693_m),
frc::Translation2d(7.41364944705348_m, 0_m),
frc::Translation2d(6.84555370398424_m, 0.191732313285866_m),
frc::Translation2d(6.12123163157097_m, 0_m),
frc::Translation2d(5.3_m, -0.5_m),
frc::Translation2d(3.47248522951068_m, 0_m),
frc::Translation2d(2.20847220118164_m, 0_m),
frc::Translation2d(1.69718603241933_m, 0_m)
      },
      frc::Pose2d(0.198833510074231_m, -0.0994167550371159_m, 180_deg),
      trajectoryConfig
  );

  /*trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    // Start at the origin facing the +X direction
      // Pass through these two interior waypoints, making an 's' curve path
      {
        frc::Pose2d(0_m, 0_m, 0_deg),
        frc::Pose2d(1_m, 0_m, 0_deg),
        frc::Pose2d(2.5_m, 0_m, 0_deg),
        frc::Pose2d(3.2_m, -0.3_m, -60_deg),
        frc::Pose2d(3.2_m, -0.9_m, -120_deg),
        frc::Pose2d(2.75_m, -1.15_m, -175_deg),
        frc::Pose2d(2.25_m, -0.85_m, -250_deg),
        frc::Pose2d(2.4_m, -0.15_m, -330_deg),
        frc::Pose2d(3_m, 0_m, -360_deg)
      },
      // End 3 meters straight ahead of where we started, facing forward
      trajectoryConfig
  );*/

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
