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

class ReconfigureCommand : public frc::Command {
 public:
  ReconfigureCommand();
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

class DefaultIntakeCommand : public frc::Command {
 public:
  DefaultIntakeCommand();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
  int state;
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

class ZeroShooterPigeonCommand : public frc::Command {
 public:
  ZeroShooterPigeonCommand();
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
};

class trackCommand : public frc::Command {
 public:
  trackCommand();
  float GetDistFudgeFactor(float percieved);
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End() override;
  void Interrupted() override;
  float GetBallisticSpeed(float dist);
  float GetBallisticAngle(float dist);
  float tx, ty;
  float rP = 0;
  float rI = 0;
  float rD = 0;
  
};
