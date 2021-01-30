#include "subsystems/Subsystems.h"

ClimberSubsystem::ClimberSubsystem() : frc::Subsystem("ClimberSubsystem"),
climber(CLIMB_ID),
lift(LIFT_ID) {
    climber.SetInverted(false);
    climber.SetNeutralMode(Brake);
    lift.SetInverted(false);
    lift.SetNeutralMode(Brake);
}

void ClimberSubsystem::InitDefaultCommand() {
    SetDefaultCommand(new DefaultClimberCommand());
}