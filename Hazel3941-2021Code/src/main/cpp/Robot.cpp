/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/commands/Scheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "PIDConstants.h"


ExampleSubsystem Robot::m_subsystem;
DriveSubsystem Robot::Drive;
IntakeSubsystem Robot::Intake;
ShooterSubsystem Robot::Shooter;
frc::Compressor Robot::robotCompressor{13};
OI Robot::oi;
std::shared_ptr<NetworkTable> Robot::table;

void Robot::RobotInit() {
  m_chooser.SetDefaultOption("Default Auto", &m_defaultAuto);
  m_chooser.AddOption("Base Auto", &BaseAuto);
  m_chooser.AddOption("Path Following", &auto_PathFollowingCommand);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  robotCompressor.SetClosedLoopControl(true);
  table = NetworkTable::GetTable("limelight");
}

void Robot::RobotPeriodic() {
  Drive.UpdatedOdometry();
}

void Robot::DisabledInit() {
  Intake.extended = false;
}

void Robot::DisabledPeriodic() { frc::Scheduler::GetInstance()->Run(); }

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString code to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional commands to the
 * chooser code above (like the commented example) or additional comparisons to
 * the if-else structure below with additional strings & commands.
 */
void Robot::AutonomousInit() {
  // std::string autoSelected = frc::SmartDashboard::GetString(
  //     "Auto Selector", "Default");
  // if (autoSelected == "My Auto") {
  //   m_autonomousCommand = &m_myAuto;
  // } else {
  //   m_autonomousCommand = &m_defaultAuto;
  // }

  m_autonomousCommand = m_chooser.GetSelected();
  //m_autonomousCommand = &PathFollowingAutoCommand;
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Start();
  }
}

void Robot::AutonomousPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TeleopInit() {
  //Intake.extended = true;
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
  for(int i = 0; i < 6; i++){
    Intake.lastStates[i] = true;
  }
}

void Robot::TeleopPeriodic() { frc::Scheduler::GetInstance()->Run(); }

void Robot::TestPeriodic() {
  Robot::Drive.TankDriveVolts((units::voltage::volt_t)6, (units::voltage::volt_t) -6); // Drives forward.
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
