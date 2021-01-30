/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */

// For example to map the left and right motors, you could define the
// following variables to use with your drivetrain subsystem.
// constexpr int kLeftMotor = 1;
// constexpr int kRightMotor = 2;

// If you are using multiple modules, make sure to define both the port
// number and the module. For example you with a rangefinder:
// constexpr int kRangeFinderPort = 1;
// constexpr int kRangeFinderModule = 1;



// Stores constants and defaults for universally accessable values.



// CAN loop IDs

static constexpr int LEFT_DRIVE_CONTROLLER_ID   = 1;

static constexpr int LEFT_DRIVE_FOLLOWER_ID     = 2;

static constexpr int INDEXER_ID                 = 3;

static constexpr int CLIMB_ID                   = 4;

static constexpr int INTAKE_ID                  = 5;

static constexpr int RIGHT_DRIVE_CONTROLLER_ID  = 6;

static constexpr int RIGHT_DRIVE_FOLLOWER_ID    = 7;

static constexpr int LIFT_ID                    = 8;

static constexpr int ARM_ID                     = 9;

static constexpr int BELT_ID                    = 10;

static constexpr int SHOOTER_A_ID               = 11;

static constexpr int SHOOTER_B_ID               = 12;

static constexpr int ARM_PIGEON_ID              = 0;

// CAN loop IDs </>



// Driver Controller Mappings

static constexpr int DRIVE_CONTROLLER_LEFT_JOYSTICK_X = 0;
static constexpr int DRIVE_CONTROLLER_DRIVE_AXIS_X = DRIVE_CONTROLLER_LEFT_JOYSTICK_X;

static constexpr int DRIVE_CONTROLLER_LEFT_JOYSTICK_Y = 1;
static constexpr int DRIVE_CONTROLLER_DRIVE_AXIS_Y = DRIVE_CONTROLLER_LEFT_JOYSTICK_Y;

static constexpr int DRIVE_CONTROLLER_LEFT_TRIGGER_Y = 2;
static constexpr int DRIVE_SHIFT_AXIS = DRIVE_CONTROLLER_LEFT_TRIGGER_Y;

static constexpr int DRIVE_CONTROLLER_B_BUTTON_RED = 2;
static constexpr int DRIVE_CONTROLLER_EXTEND_RETRACT_INTAKE_BUTTON = DRIVE_CONTROLLER_B_BUTTON_RED;

static constexpr int DRIVE_CONTROLLER_X_BUTTON_BLUE = 3;
static constexpr int DRIVE_CONTROLLER_ENABLE_INTAKEINDEXING_BUTTON = DRIVE_CONTROLLER_X_BUTTON_BLUE;

static constexpr int DRIVE_CONTROLLER_LEFT_BUMPER  = 5;
static constexpr int DRIVE_TRACK_BUTTON = DRIVE_CONTROLLER_LEFT_BUMPER;

static constexpr int DRIVE_CONTROLLER_RIGHT_BUMPER  = 6;
static constexpr int DRIVE_ARM_BUTTON = DRIVE_CONTROLLER_RIGHT_BUMPER;

static constexpr int DRIVE_CONTROLLER_A_BUTTON_GREEN = 1;
static constexpr int DRIVE_CONTROLLER_ROLL_SHOOTER_BACKWARD = DRIVE_CONTROLLER_A_BUTTON_GREEN;

static constexpr int DRIVE_CONTROLLER_Y_BUTTON_YELLOW = 4;
static constexpr int DRIVE_CONTROLLER_ROLL_SHOOTER_FORWARD = DRIVE_CONTROLLER_Y_BUTTON_YELLOW;

//static constexpr int TRACK_BUTTON = 5;

// Driver Controller Mappings </>


// Operator Controller Mappings

static constexpr int OPERATOR_JOYSTICK_INNERRING_TOP = 8;
static constexpr int MANUAL_OPERATOR_OVERRIDE_BUTTON = OPERATOR_JOYSTICK_INNERRING_TOP;

static constexpr bool OPERATOR_JOYSTICK_FORWARDREVERSE_FLIP = true;
static constexpr int OPERATOR_JOYSTICK_FORWARDREVERSE_AXIS_ID = 1;

static constexpr int OPERATOR_JOYSTICK_LEFTRIGHT_AXIS_ID = 0;

static constexpr int OPERATOR_JOYSTICK_THROTTLE_AXIS_ID = 3;
static constexpr int OPERATOR_SHOOTER_SPEED_AXIS_ID = OPERATOR_JOYSTICK_THROTTLE_AXIS_ID;

static constexpr bool OPERATOR_SHOOTER_SPEED_AXIS_FLIP = false;

static constexpr int OPERATOR_JOYSTICK_TRIGGER_BUTTON = 1;
static constexpr int OPERATOR_SHOOTER_SPIN_ENABLE_BUTTON = OPERATOR_JOYSTICK_TRIGGER_BUTTON;

static constexpr int OPERATOR_JOYSTICK_HAT_RIGHT_TOP = 6;
static constexpr int OPERATOR_INDEXER_FEED_FORWARD_BUTTON = OPERATOR_JOYSTICK_HAT_RIGHT_TOP;

static constexpr int OPERATOR_JOYSTICK_HAT_RIGHT_BOTTOM = 4;
static constexpr int OPERATOR_INDEXER_FEED_REVERSE_BUTTON = OPERATOR_JOYSTICK_HAT_RIGHT_BOTTOM;

static constexpr int OPERATOR_JOYSTICK_HAT_LEFT_TOP = 5;
static constexpr int OPERATOR_INTAKE_REJECT_BUTTON = OPERATOR_JOYSTICK_HAT_LEFT_TOP;

static constexpr int OPERATOR_JOYSTICK_HAT_LEFT_BOTTOM = 3;
static constexpr int OPERATOR_INTAKE_FEED_ROBOT_BUTTON = OPERATOR_JOYSTICK_HAT_LEFT_BOTTOM;

static constexpr int OPERATOR_JOYSTICK_INNERRING_MIDDLE = 10;
static constexpr int OPERATOR_LIFT_UP_BUTTON = OPERATOR_JOYSTICK_INNERRING_MIDDLE;

static constexpr int OPERATOR_JOYSTICK_INNERRING_BOTTOM = 12;
static constexpr int OPERATOR_LIFT_DOWN_BUTTON = OPERATOR_JOYSTICK_INNERRING_BOTTOM;

static constexpr int OPERATOR_JOYSTICK_OUTERRING_TOP = 7;

static constexpr int OPERATOR_JOYSTICK_OUTERRING_MIDDLE = 9;
static constexpr int OPERATOR_CLIMB_UP_BUTTON = OPERATOR_JOYSTICK_OUTERRING_MIDDLE;

static constexpr int OPERATOR_JOYSTICK_OUTERRING_BOTTOM = 11;
static constexpr int OPERATOR_CLIMB_RELEASE_BUTTON = OPERATOR_JOYSTICK_OUTERRING_BOTTOM;

// Operator Controller Mappings </>



// (it was I,) DIO Mappings

static constexpr int DIO_INDEXER_1A = 0;

static constexpr int DIO_INDEXER_1B = 1;

static constexpr int DIO_INDEXER_2 = 2;

static constexpr int DIO_INDEXER_3 = 3;

static constexpr int DIO_INDEXER_4 = 4;

static constexpr int DIO_INDEXER_5 = 5;

static constexpr int DIO_SHOOTER_1 = 6;

static constexpr int DIO_SHOOTER_2 = 7;

static constexpr int DIO_UPPER_LIM = 8;

static constexpr int DIO_LOWER_LIM = 9;

// DIO Mappings </>



// Pneumatics Mapping

static constexpr int SHIFT_UP = 0;

static constexpr int SHIFT_DOWN = 0;

static constexpr int EXTEND_INTAKE_LEFT = 7;

static constexpr int RETRACT_INTAKE_LEFT = 6;

static constexpr int EXTEND_INTAKE_RIGHT = 3;

static constexpr int RETRACT_INTAKE_RIGHT = 2;

// Pneumatics Mapping </>



// PID Defaults



// PID Defaults </>