/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Command.h>
#include "frc/smartdashboard/SmartDashboard.h"

class ExampleCommand : public frc::Command {
 public:
  ExampleCommand();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};

class DefaultDriveCommand : public frc::Command {
 public:
  DefaultDriveCommand();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};

class DefaultIndexCommand : public frc::Command {
 public:
  DefaultIndexCommand();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};

class DefaultIntakeCommand : public frc::Command {
 public:
  DefaultIntakeCommand();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};

class DefaultShooterCommand : public frc::Command {
 public:
  DefaultShooterCommand();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};

class DefaultClimberCommand : public frc::Command {
 public:
  DefaultClimberCommand();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};

class trackCommand : public frc::Command {
 public:
  trackCommand();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
  float tx, ty;
  float P;
  float I;
  float D;
  float PC = 0.1;
  float IC = 0.01;
  float DC = 0.05;
};
