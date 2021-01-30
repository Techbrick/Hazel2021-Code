#include "subsystems/Subsystems.h"
#include "PIDConstants.h"

ShooterSubsystem::ShooterSubsystem() : frc::Subsystem("ShooterSubsystem"),
armMotor(ARM_ID),
shooterController(SHOOTER_A_ID),
shooterFollower(SHOOTER_B_ID),
pigeon(ARM_PIGEON_ID)
{
  armMotor.ConfigFactoryDefault();
  armMotor.SetInverted(false);
  armMotor.SetNeutralMode(Brake);

  pigeon.ConfigFactoryDefault();

  // What I think this is doing: configuring feedback device 0 
  armMotor.ConfigRemoteFeedbackFilter(pigeon.GetDeviceNumber(), ctre::phoenix::motorcontrol::RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw, 0, kTimeoutMs);
  armMotor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::RemoteSensor0, ARM_ANGLE_CAN_LOOP_ID, kTimeoutMs);

  armMotor.Config_kF(ARM_ANGLE_CAN_LOOP_ID, ARM_ANGLE_F, kTimeoutMs);
  armMotor.Config_kP(ARM_ANGLE_CAN_LOOP_ID, ARM_ANGLE_P, kTimeoutMs);
  armMotor.Config_kI(ARM_ANGLE_CAN_LOOP_ID, ARM_ANGLE_I, kTimeoutMs);
  armMotor.Config_kD(ARM_ANGLE_CAN_LOOP_ID, ARM_ANGLE_D, kTimeoutMs);
  /*armMotor.ConfigReverseLimitSwitchSource(
					LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
					LimitSwitchNormal::LimitSwitchNormal_NormallyClosed);
  */
  shooterFollower.ConfigFactoryDefault();
  shooterFollower.SetInverted(false);
  shooterFollower.SetNeutralMode(Coast);

  shooterController.ConfigFactoryDefault();
  shooterController.SetInverted(true);
  shooterController.SetNeutralMode(Coast);
  
  shooterFollower.Follow(shooterController);
}

void ShooterSubsystem::InitDefaultCommand() {
  SetDefaultCommand(new DefaultShooterCommand());
}