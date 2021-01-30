#include "subsystems/Subsystems.h"

IndexSubsystem::IndexSubsystem() : frc::Subsystem("IndexSubsystem"),
indexWheelMotor(INDEXER_ID),
beltMotor(BELT_ID) {
  indexWheelMotor.SetInverted(true);
  indexWheelMotor.SetNeutralMode(Coast);
  beltMotor.SetInverted(false);
  beltMotor.SetNeutralMode(Coast);
}

void IndexSubsystem::InitDefaultCommand() {
  SetDefaultCommand(new DefaultIndexCommand());
}