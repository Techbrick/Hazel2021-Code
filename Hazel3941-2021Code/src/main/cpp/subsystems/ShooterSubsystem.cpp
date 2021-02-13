#include "subsystems/Subsystems.h"
#include "PIDConstants.h"

ShooterSubsystem::ShooterSubsystem() : frc::Subsystem("ShooterSubsystem"),
armMotor(ARM_ID),
left(SHOOTER_A_ID),
right(SHOOTER_B_ID),
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
  left.ConfigFactoryDefault();
  left.SetInverted(false);
  left.SetNeutralMode(Coast);

  right.ConfigFactoryDefault();
  right.SetInverted(true);
  right.SetNeutralMode(Coast);
  
  // shooterFollower.Follow(shooterController);

  left.ConfigSelectedFeedbackSensor(0);
  right.ConfigSelectedFeedbackSensor(0);
}

void ShooterSubsystem::InitDefaultCommand() {
  SetDefaultCommand(new DefaultShooterCommand());
}