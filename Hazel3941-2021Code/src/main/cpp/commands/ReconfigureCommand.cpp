/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/Commands.h"

#include "PIDConstants.h"
#include "Robot.h"

ReconfigureCommand::ReconfigureCommand() {
    // Use Requires() here to declare subsystem dependencies
    Requires(&Robot::Shooter);
    Requires(&Robot::Drive);
    Requires(&Robot::Indexer);
    Requires(&Robot::Intake);
    //frc::SmartDashboard::PutNumber("VTrack P", ARM_ANGLE_P);
    //frc::SmartDashboard::PutNumber("VTrack I", ARM_ANGLE_I);
    //frc::SmartDashboard::PutNumber("VTrack D", ARM_ANGLE_D);
    //frc::SmartDashboard::PutNumber("VTrack F", ARM_ANGLE_F);
}

// Called just before this Command runs the first time
void ReconfigureCommand::Initialize() {
    
}

// Called repeatedly when this Command is scheduled to run
void ReconfigureCommand::Execute() {
    // Update globals from shuffleboard
    ARM_ANGLE_P = frc::SmartDashboard::GetNumber("VTrackP", ARM_ANGLE_P);
    ARM_ANGLE_I = frc::SmartDashboard::GetNumber("VTrackI", ARM_ANGLE_I);
    ARM_ANGLE_D = frc::SmartDashboard::GetNumber("VTrackD", ARM_ANGLE_D);
    ARM_ANGLE_F = frc::SmartDashboard::GetNumber("VTrackF", ARM_ANGLE_F);
    
    Robot::Shooter.armMotor.Config_kF(ARM_ANGLE_CAN_LOOP_ID, ARM_ANGLE_F, kTimeoutMs);
    Robot::Shooter.armMotor.Config_kP(ARM_ANGLE_CAN_LOOP_ID, ARM_ANGLE_P, kTimeoutMs);
    Robot::Shooter.armMotor.Config_kI(ARM_ANGLE_CAN_LOOP_ID, ARM_ANGLE_I, kTimeoutMs);
    Robot::Shooter.armMotor.Config_kD(ARM_ANGLE_CAN_LOOP_ID, ARM_ANGLE_D, kTimeoutMs);
}

// Make this return true when this Command no longer needs to run execute()
bool ReconfigureCommand::IsFinished() { return true; }

// Called once after isFinished returns true
void ReconfigureCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReconfigureCommand::Interrupted() {}
