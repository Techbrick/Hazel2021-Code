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
    bool invertleft = true;
    bool invertright = true;

    LeftController.ConfigFactoryDefault();
    LeftController.ClearStickyFaults();
    LeftController.SetNeutralMode(Brake);
    LeftController.SetInverted(invertleft);  // Setting this 03/20/21

    LeftController.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 10);
    LeftController.SetSensorPhase(false); // Setting this 03/20/21

    RightController.ConfigFactoryDefault();
    RightController.ClearStickyFaults();
    RightController.SetNeutralMode(Brake);
    RightController.SetInverted(invertright); // Setting this 03/20/21

    RightController.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 10);
    RightController.SetSensorPhase(true); // Setting this 03/20/21

    LeftFollower.ConfigFactoryDefault();
    LeftFollower.ClearStickyFaults();
    LeftFollower.SetNeutralMode(Brake);
    LeftFollower.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
    LeftFollower.Follow(LeftController);

    RightFollower.ConfigFactoryDefault();
    RightFollower.ClearStickyFaults();
    RightFollower.SetNeutralMode(Brake);
    RightFollower.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
    RightFollower.Follow(RightController);

// caitlyn's attempt to no kidding reset the pose.
    ResetOdometryPose();
}

void DriveSubsystem::ResetOdometryPose(){
    LeftController.SetSelectedSensorPosition(0,0,10);
    RightController.SetSelectedSensorPosition(0,0,10);
    navx.Reset();
    odometry.ResetPosition(frc::Pose2d(),frc::Rotation2d());
    LeftController.SetSelectedSensorPosition(0,0,10);
    RightController.SetSelectedSensorPosition(0,0,10);
}

void DriveSubsystem::InitDefaultCommand() {
    SetDefaultCommand(new DefaultDriveCommand());
}

void DriveSubsystem::UpdatedOdometry(){
    // If the robot is moving forward in a straight line, both distances (left and right) must be positive.
    // UNinverted this 
    units::meter_t leftmeters = units::meter_t( LeftController.GetSelectedSensorPosition() * kEncoderDistancePerPulse); 
    units::meter_t rightmeters = units::meter_t(RightController.GetSelectedSensorPosition() * kEncoderDistancePerPulse);
    odometry.Update(navx.GetRotation2d(), leftmeters, rightmeters);
    /*
    frc::SmartDashboard::PutNumber("OdoX", odometry.GetPose().X().value());
    frc::SmartDashboard::PutNumber("OdoY", odometry.GetPose().Y().value());
    frc::SmartDashboard::PutNumber("LeftMeters", leftmeters.value());
    frc::SmartDashboard::PutNumber("RightMeters", rightmeters.value());
    frc::SmartDashboard::PutNumber("NavX", navx.GetRotation2d().Degrees().value());
    */
}

frc::Pose2d DriveSubsystem::GetPose() {
    return odometry.GetPose();
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
    LeftController.SetVoltage(left);
    RightController.SetVoltage(right);
    frc::SmartDashboard::PutNumber("VoltageLeft", left.value());
    frc::SmartDashboard::PutNumber("VoltageRight", -right.value());
    driveControl.Feed();
}

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
    // This block contains a scary magic number, 0.1, that's because
    // the Talon SRX reports in units per 100ms which we can't use.
    //frc::SmartDashboard::PutNumber("Left Controller (mps?)", LeftController.GetSelectedSensorVelocity(0) * 40960 * kEncoderDistancePerPulse); // constant is a magic #!
    //frc::SmartDashboard::PutNumber("Right Controller (mps?)", RightController.GetSelectedSensorVelocity(0) * 40960 * kEncoderDistancePerPulse);
    //frc::SmartDashboard::PutNumber("Left Controller (raw)", LeftController.GetSelectedSensorVelocity(0));
    //frc::SmartDashboard::PutNumber("Right Controller (raw)", RightController.GetSelectedSensorVelocity(0));
    return {
         
        units::meters_per_second_t(LeftController.GetSelectedSensorVelocity(0) * 10 * kEncoderDistancePerPulse ), // 
        units::meters_per_second_t(RightController.GetSelectedSensorVelocity(0) * 10 * kEncoderDistancePerPulse)  // * 4096 * kEncoderDistancePerPulse
        
       //units::meters_per_second_t(0),
       //units::meters_per_second_t(0)
    };
}
