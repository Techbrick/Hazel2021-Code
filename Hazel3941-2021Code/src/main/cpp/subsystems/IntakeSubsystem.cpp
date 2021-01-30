#include "subsystems/Subsystems.h"

IntakeSubsystem::IntakeSubsystem() : frc::Subsystem("IntakeSubsystem"),
intakeMotor(INTAKE_ID) {
  intakeMotor.SetInverted(false);
  intakeMotor.SetNeutralMode(Coast);
}

void IntakeSubsystem::InitDefaultCommand() {
  SetDefaultCommand(new DefaultIntakeCommand());
}