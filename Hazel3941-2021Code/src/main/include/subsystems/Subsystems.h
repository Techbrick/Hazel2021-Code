/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/commands/Subsystem.h>
#include "commands/Commands.h"
#include "ctre/Phoenix.h"
#include "frc/drive/DifferentialDrive.h"
#include "frc/DigitalInput.h"
#include "frc/Compressor.h"
#include "frc/DoubleSolenoid.h"
#include "Objects.h"
#include "RobotMap.h"

class ExampleSubsystem : public frc::Subsystem {
  public:
    ExampleSubsystem();
    void InitDefaultCommand() override;

  private:
  // It's desirable that everything possible under private except
  // for methods that implement subsystem capabilities
};

class ClimberSubsystem : public frc::Subsystem {
  public:
    ClimberSubsystem();
    TalonSRX climber;
    TalonSRX lift;
    void InitDefaultCommand() override;

  private:

};

class DriveSubsystem : public frc::Subsystem {
  public:
    DriveSubsystem();
    void InitDefaultCommand() override;
    // Remove WPI_ for motion magic or something
    WPI_TalonSRX RightController;
    WPI_VictorSPX RightFollower;
    WPI_TalonSRX LeftController;
    WPI_VictorSPX LeftFollower;
    frc::DifferentialDrive driveControl{LeftController, RightController};
    frc::DoubleSolenoid ShifterSolenoid {13, 0, 1};
    
  private:

};

class IndexSubsystem : public frc::Subsystem {
  public:
    IndexSubsystem();
    void InitDefaultCommand() override;
    bool beltOn = false;
    bool indexWheelOn = false;
    bool lastStates[6] = {true, true, true, true, true, true};
    TalonSRX indexWheelMotor;
    TalonSRX beltMotor;
    frc::DigitalInput distanceA{DIO_INDEXER_1A};
    frc::DigitalInput distanceB{DIO_INDEXER_1B};
    frc::DigitalInput beltA{DIO_INDEXER_2};
    frc::DigitalInput beltB{DIO_INDEXER_3};
    frc::DigitalInput beltC{DIO_INDEXER_4};
    frc::DigitalInput beltD{DIO_INDEXER_5};
    std::vector<Ball> balls;
    bool drivenManually = false;

  private:

};

class IntakeSubsystem : public frc::Subsystem {
  public:
    IntakeSubsystem();
    void InitDefaultCommand() override;
    TalonSRX intakeMotor;
    frc::DoubleSolenoid ExtenderSolenoidA {13, 2, 3};
    frc::DoubleSolenoid ExtenderSolenoidB {13, 6, 7};
    bool indexEnabled = true;
    bool manualEnabled = false;
    bool buttonLastA = false;
    bool buttonLastB = false;
    bool extended = false;
  private:

};

class ShooterSubsystem : public frc::Subsystem {
  public:
    ShooterSubsystem();
    void InitDefaultCommand() override;
    TalonSRX armMotor;
    TalonSRX shooterController;
    TalonSRX shooterFollower;
    PigeonIMU pigeon;
    //frc::DigitalInput upperLim{DIO_UPPER_LIM};
    frc::DigitalInput lowerLim{DIO_LOWER_LIM};
    bool drivenManually = false;
  private: 
};
