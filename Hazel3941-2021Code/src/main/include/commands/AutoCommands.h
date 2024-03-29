/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include "RobotCharacteristics.h"
#include "frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/trajectory/TrajectoryConfig.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/trajectory/Trajectory.h"
#include "frc2/command/RamseteCommand.h"

class baseAutoCommand : public frc::Command {
 public:
  baseAutoCommand();
  int timer;
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};

class PathFollowingAutoCommand : public frc::Command {
 public:
  PathFollowingAutoCommand();
  int timer;
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
  frc::DifferentialDriveKinematics kDriveKinematics = frc::DifferentialDriveKinematics(kTrackwidth);
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint = frc::DifferentialDriveVoltageConstraint(
    frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
    kDriveKinematics,
    10_V
  );
  frc::TrajectoryConfig trajectoryConfig = frc::TrajectoryConfig(kMaxSpeed, kMaxAcceleration);
  frc::Trajectory trajectory;
  frc2::RamseteCommand* ramseteCommand = nullptr;
};
