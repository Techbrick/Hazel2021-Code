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

  armMotor.SetSensorPhase(true);
  /*armMotor.ConfigReverseLimitSwitchSource(
					LimitSwitchSource::LimitSwitchSource_FeedbackConnector,
					LimitSwitchNormal::LimitSwitchNormal_NormallyClosed);
  */

  left.ConfigFactoryDefault();
  left.SetInverted(TalonFXInvertType::CounterClockwise);
  left.SetNeutralMode(Coast);

  right.ConfigFactoryDefault();
  right.SetInverted(TalonFXInvertType::Clockwise);
  right.SetNeutralMode(Coast);
  
  // shooterFollower.Follow(shooterController);

  left.ConfigNominalOutputForward(0, 10);
  left.ConfigNominalOutputReverse(0, 10);
  left.ConfigPeakOutputForward(1, 10);
  left.ConfigPeakOutputReverse(-1, 10);

  right.ConfigNominalOutputForward(0, 10);
  right.ConfigNominalOutputReverse(0, 10);
  right.ConfigPeakOutputForward(1, 10);
  right.ConfigPeakOutputReverse(-1, 10);

  left.SelectProfileSlot(0, 0);
  right.SelectProfileSlot(0, 0);

  left.Config_kF(0, 0.04797, kTimeoutMs);
	left.Config_kP(0, 0.052, kTimeoutMs);
	left.Config_kI(0, 0.0, kTimeoutMs);
	left.Config_kD(0, 0.0, kTimeoutMs);

  right.Config_kF(0, 0.04797, kTimeoutMs);
  right.Config_kP(0, 0.052, kTimeoutMs);
	right.Config_kI(0, 0.0, kTimeoutMs);
	right.Config_kD(0, 0.0, kTimeoutMs);

  /*left.ConfigMotionCruiseVelocity(1500, 10);
  right.ConfigMotionCruiseVelocity(1500, 10);

  left.ConfigMotionAcceleration(1500, 10);
  right.ConfigMotionAcceleration(1500, 10);*/

  left.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
  right.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);

  left.SetSelectedSensorPosition(0, 0, 10);
  right.SetSelectedSensorPosition(0, 0, 10);
}

void ShooterSubsystem::InitDefaultCommand() {
  SetDefaultCommand(new DefaultShooterCommand());
}