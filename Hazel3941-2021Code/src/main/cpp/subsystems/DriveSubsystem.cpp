/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Subsystems.h"
#include "Robot.h"
#include "RobotCharacteristics.h"

DriveSubsystem::DriveSubsystem() : frc::Subsystem("DriveSubsystem"),
LeftController(LEFT_DRIVE_CONTROLLER_ID),
LeftFollower(LEFT_DRIVE_FOLLOWER_ID),
RightController(RIGHT_DRIVE_CONTROLLER_ID),
RightFollower(RIGHT_DRIVE_FOLLOWER_ID),
odometry(frc::Rotation2d(units::degree_t(Robot::navx->GetPitch())))/*,
driveControl(LeftController, RightController)*/
{    
    LeftController.ConfigFactoryDefault();
    LeftController.ClearStickyFaults();
    LeftController.SetNeutralMode(Brake);

    LeftController.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 10);

    LeftFollower.ConfigFactoryDefault();
    LeftFollower.ClearStickyFaults();
    LeftFollower.SetNeutralMode(Brake);
    LeftFollower.Follow(LeftController);

    RightController.ConfigFactoryDefault();
    RightController.ClearStickyFaults();
    RightController.SetNeutralMode(Brake);

    RightController.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 10);

    RightFollower.ConfigFactoryDefault();
    RightFollower.ClearStickyFaults();
    RightFollower.SetNeutralMode(Brake);
    RightFollower.Follow(RightController);

    ResetEncoders();
}

void DriveSubsystem::InitDefaultCommand() {
    SetDefaultCommand(new DefaultDriveCommand());
}

void DriveSubsystem::Periodic() {
    odometry.Update(
        frc::Rotation2d(units::degree_t(Robot::navx->GetPitch())),
        units::meter_t(LeftController.GetSelectedSensorPosition() * kEncoderDistancePerPulse),
        units::meter_t(RightController.GetSelectedSensorPosition() * kEncoderDistancePerPulse)
    );
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
    LeftController.SetVoltage(left);
    RightController.SetVoltage(right);
    driveControl.Feed();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
    ResetEncoders();
    odometry.ResetPosition(
        pose,
        frc::Rotation2d(units::degree_t(Robot::navx->GetPitch()))
    );
}

void DriveSubsystem::ResetEncoders() {
    LeftController.SetSelectedSensorPosition(0, 0, 10);
    RightController.SetSelectedSensorPosition(0, 0, 10);
}

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
    // This block contains a scary magic number, 0.1, that's because
    // the Talon SRX reports in units per 100ms which we can't use.
    return {
        units::meters_per_second_t(LeftController.GetSelectedSensorVelocity(0) * 0.1 * kEncoderDistancePerPulse),
        units::meters_per_second_t(RightController.GetSelectedSensorVelocity(0) * 0.1 * kEncoderDistancePerPulse)
    };
}