#include "subsystems/Subsystems.h"

IntakeSubsystem::IntakeSubsystem() : frc::Subsystem("IntakeSubsystem"),
indexWheelMotor(INDEXER_ID),
beltMotor(BELT_ID),
intakeMotor(INTAKE_ID) {
  intakeMotor.SetInverted(false);
  intakeMotor.SetNeutralMode(Coast);

  indexWheelMotor.SetInverted(true);
  indexWheelMotor.SetNeutralMode(Coast);
  beltMotor.SetInverted(false);
  beltMotor.SetNeutralMode(Coast);
}

void IntakeSubsystem::InitDefaultCommand() {
  SetDefaultCommand(new DefaultIntakeCommand());
}