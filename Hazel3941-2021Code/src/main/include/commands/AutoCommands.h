/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include "frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h"
#include "RobotCharacteristics.h"

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

class TestTrajectoryCommand : public frc::Command {
 public:
  TestTrajectoryCommand();
  frc::DifferentialDriveVoltageConstraint autoVoltageConstraint {
    frc::SimpleMotorFeedforward<units::meters>(ks, kv, ka),
    kDriveKinematics,
    10_V
  };
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};
