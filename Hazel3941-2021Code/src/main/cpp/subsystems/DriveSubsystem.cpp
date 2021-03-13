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

// caitlyn's attempt to no kidding reset the pose.
    ResetOdometryPose();
}

void DriveSubsystem::ResetOdometryPose(){
    LeftController.SetSelectedSensorPosition(0,0,10);
    RightController.SetSelectedSensorPosition(0,0,10);
    odometry.ResetPosition(frc::Pose2d(),frc::Rotation2d());
    LeftController.SetSelectedSensorPosition(0,0,10);
    RightController.SetSelectedSensorPosition(0,0,10);
}

void DriveSubsystem::InitDefaultCommand() {
    SetDefaultCommand(new DefaultDriveCommand());
}

void DriveSubsystem::UpdatedOdometry(){
    // If the robot is moving forward in a straight line, both distances (left and right) must be positive.
    units::meter_t leftmeters = units::meter_t( -LeftController.GetSelectedSensorPosition() * kEncoderDistancePerPulse); // Inverting for odometry purposes
    units::meter_t rightmeters = units::meter_t(RightController.GetSelectedSensorPosition() * kEncoderDistancePerPulse);
    odometry.Update(
        navx.GetRotation2d(), leftmeters, rightmeters);
    frc::SmartDashboard::PutNumber("OdoX", odometry.GetPose().X().value());
    frc::SmartDashboard::PutNumber("OdoY", odometry.GetPose().Y().value());
    frc::SmartDashboard::PutNumber("LeftMeters", leftmeters.value());
    frc::SmartDashboard::PutNumber("RightMeters", rightmeters.value());
}
