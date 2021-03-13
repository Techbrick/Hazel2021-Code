#include <frc/kinematics/DifferentialDriveKinematics.h>

#include <wpi/math>
#include <cmath>

#include <units/length.h>
#include <units/voltage.h>
#include <units/time.h>
#include <units/acceleration.h>

#pragma once

constexpr auto ks = 1.6_V;
constexpr auto kv = 3.17 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.668 * 1_V * 1_s * 1_s / 1_m;

constexpr double kPDriveVel = 2.27;

constexpr auto kTrackwidth = 1.435676591_m;
extern const frc::DifferentialDriveKinematics kDriveKinematics;

constexpr auto kMaxSpeed = 0.73_mps;
constexpr auto kMaxAcceleration = 2.92_mps_sq;

constexpr double kRamseteB = 2;
constexpr double kRamseteZeta = 0.7;

// Multiply raw sensor units by this to get distance traveled:
constexpr double kEncoderDistancePerPulse = (0.15 * M_PI) / 4096.0;