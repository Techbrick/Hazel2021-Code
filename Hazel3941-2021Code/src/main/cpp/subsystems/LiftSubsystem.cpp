#include "subsystems/Subsystems.h"
#include "RobotMap.h"

LiftSubsystem::LiftSubsystem() : frc::Subsystem("LiftSubsystem"),
winchMotor(WINCH_ID) {
    // TODO: init winch motor settings with appropriate breaking mode + direction to climb. (See drive subsystem for motor init examples)
}

void ExampleSubsystem::InitDefaultCommand() {
  // Set the default command for a subsystem here.
  SetDefaultCommand(new DefaultLiftCommand());
}