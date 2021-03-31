#include <frc/kinematics/DifferentialDriveKinematics.h>

#include <wpi/math>
#include <cmath>

#include <units/length.h>
#include <units/voltage.h>
#include <units/time.h>
#include <units/acceleration.h>
#include <units/velocity.h>

#pragma once

constexpr auto ks = 1.6_V;
constexpr auto kv = 3.17 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.668 * 1_V * 1_s * 1_s / 1_m;

constexpr double kPDriveVel = 2.27;

//constexpr auto kTrackwidth = 1.435676591_m;
constexpr auto kTrackwidth = 1_m;

// extern const frc::DifferentialDriveKinematics kDriveKinematics;

constexpr auto kMaxSpeed = 1.5_mps;
constexpr auto kMaxAcceleration = 10_mps_sq;

constexpr double kRamseteB = 2;
constexpr double kRamseteZeta = 0.7;

// Multiply raw sensor units by this to get distance traveled:
//constexpr double kEncoderDistancePerPulse = (0.1524 * M_PI) / 4096.0;
// Third stage gear box = 30:54
// 6in wheel = .1524 m
// 4096 = encoder ticks for a mag encoder per revolution
// 3 = VEX ball shifter encoder shaft spins 3x faster than the 30 tooth output shaft
constexpr double kEncoderDistancePerPulse = 30.0 / 54.0 * .1524 * M_PI / 4096.0 / 3.0; // 1 tick = 0.00002164553 m travelled