#include "commands/Commands.h"

#include "Robot.h"

DefaultLiftCommand::DefaultLiftCommand() {
  // Use Requires() here to declare subsystem dependencies
  Requires(&Robot::Lift);
}

// Called just before this Command runs the first time
void DefaultLiftCommand::Initialize() {
}

// Called repeatedly when this Command is scheduled to run
void DefaultLiftCommand::Execute() {

    // TODO: write logic to release latch when OPERATOR_LATCH_RELEASE_BUTTON pressed + run winch when OPERATOR_DRIVE_WINCH_BUTTON pressed
    // Note: might want to time limit the winch drive to prevent over driving + add some sort of latch return control for resetting between rounds. 


}

// Make this return true when this Command no longer needs to run execute()
bool DefaultLiftCommand::IsFinished() { return false; }

// Called once after isFinished returns true
void DefaultLiftCommand::End() {}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DefaultLiftCommand::Interrupted() {}
