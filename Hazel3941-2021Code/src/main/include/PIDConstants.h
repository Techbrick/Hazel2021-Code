#pragma once

/**
 * Map of PID Constants for PID Tuning
 */


// General constants

static constexpr int kTimeoutMs = 40;

// Loop specific constants

static constexpr int ARM_ANGLE_CAN_LOOP_ID = 0;
static float ARM_ANGLE_F = 0.0;
static float ARM_ANGLE_P = 3.0;
static float ARM_ANGLE_I = 0.0;
static float ARM_ANGLE_D = 0.0;
static float TARGETTILTANGLE = 0.0;

static constexpr float TRACK_HORIZONTAL_P = 0.014095239177346;
static constexpr float TRACK_HORIZONTAL_I = 0.023391534850001;
static constexpr float TRACK_HORIZONTAL_D = 0.0;



