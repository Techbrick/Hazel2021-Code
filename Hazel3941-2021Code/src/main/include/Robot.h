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

#include "OI.h"
#include "commands/Commands.h"
#include "commands/AutoCommands.h"
#include "subsystems/Subsystems.h"

class Robot : public frc::TimedRobot {
 public:
  static ExampleSubsystem m_subsystem;
  static ClimberSubsystem Climber;
  static DriveSubsystem Drive;
  static IndexSubsystem Indexer;
  static IntakeSubsystem Intake;
  static ShooterSubsystem Shooter;
  static OI oi;
  static frc::Compressor robotCompressor;
  static std::shared_ptr<NetworkTable> table;

  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc::Command* m_autonomousCommand = nullptr;
  ExampleCommand m_defaultAuto;
  baseAutoCommand BaseAuto;
  frc::SendableChooser<frc::Command*> m_chooser;
};
