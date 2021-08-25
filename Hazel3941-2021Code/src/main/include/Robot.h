/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <frc/commands/Command.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/SPI.h>

#include "OI.h"
#include "commands/Commands.h"
#include "commands/AutoCommands.h"
#include "subsystems/Subsystems.h"

class Robot : public frc::TimedRobot {
 public:
  static ExampleSubsystem m_subsystem;
  static LiftSubsystem Lift;
  static DriveSubsystem Drive;
  static IntakeSubsystem Intake;
  static ShooterSubsystem Shooter;
  static TrajectoryTimingSubsystem TrajectoryTiming;
  static OI oi;
  static frc::Compressor robotCompressor;
  static std::shared_ptr<NetworkTable> table;

  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledPeriodic() override;
  void DisabledInit() override;
  void AutonomousInit() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc::Command* m_autonomousCommand = &auto_PathFollowingCommand;
  ExampleCommand m_defaultAuto;
  baseAutoCommand BaseAuto;
  PathFollowingAutoCommand auto_PathFollowingCommand;
  frc::SendableChooser<frc::Command*> m_chooser;
};
