/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Subsystems.h"

DriveSubsystem::DriveSubsystem() : frc::Subsystem("DriveSubsystem"),
LeftController(LEFT_DRIVE_CONTROLLER_ID),
LeftFollower(LEFT_DRIVE_FOLLOWER_ID),
RightController(RIGHT_DRIVE_CONTROLLER_ID),
RightFollower(RIGHT_DRIVE_FOLLOWER_ID), 
odometry(navx.GetRotation2d())
/*,*/
{
    LeftController.ConfigFactoryDefault();
    LeftController.ClearStickyFaults();
    LeftController.SetNeutralMode(Brake);

    LeftController.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 10);

    RightController.ConfigFactoryDefault();
    RightController.ClearStickyFaults();
    RightController.SetNeutralMode(Brake);

    RightController.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 10);

    LeftFollower.ConfigFactoryDefault();
    LeftFollower.ClearStickyFaults();
    LeftFollower.SetNeutralMode(Brake);
    LeftFollower.Follow(LeftController);

    RightFollower.ConfigFactoryDefault();
    RightFollower.ClearStickyFaults();
    RightFollower.SetNeutralMode(Brake);
    RightFollower.Follow(RightController);
}

void DriveSubsystem::InitDefaultCommand() {
    SetDefaultCommand(new DefaultDriveCommand());
}

void DriveSubsystem::UpdatedOdometry(){
    odometry.Update(
        navx.GetRotation2d(),
        units::meter_t(LeftController.GetSelectedSensorPosition() * kEncoderDistancePerPulse),
        units::meter_t(RightController.GetSelectedSensorPosition() * kEncoderDistancePerPulse)
    );
    frc::SmartDashboard::PutNumber("NAVX", odometry.GetPose().X().value());
    frc::SmartDashboard::PutNumber("NAVY", odometry.GetPose().Y().value());
}
