#pragma once

#include "main.h"

// Robot dimensions
constexpr QLength WHEEL_TRACK                  = 12.63_in;
constexpr QLength TRACKING_WHEEL_DIAMETER      = 2.75_in;
constexpr QLength TRACKING_BACK_WHEEL_DISTANCE = 5.7_in;

// Drive profiling constants
constexpr double DRIVE_MAX_VEL   = 0.5;
constexpr double DRIVE_MAX_ACCEL = 0.5;
constexpr double DRIVE_MAX_JERK  = 8.0;

// Drivetrain settling tolerances
constexpr double DRIVE_ENCODER_TARGET_ERROR = 40;
constexpr double DRIVE_ENCODER_TARGET_DERIV = 5;
constexpr QTime DRIVE_ENCODER_TARGET_TIME   = 200_ms;

constexpr double STRAFING_DIST_TARGET_ERROR = 0.03;
constexpr double STRAFING_DIST_TARGET_DERIV = 5;
constexpr QTime STRAFING_DIST_TARGET_TIME   = 100_ms;

constexpr double STRAFING_ANGLE_TARGET_ERROR = 1.0;
constexpr double STRAFING_ANGLE_TARGET_DERIV = 5;
constexpr QTime STRAFING_ANGLE_TARGET_TIME   = 100_ms;

constexpr QLength DISTANCE_BEFORE_MOVE = 5_cm;
constexpr QAngle ANGLE_BEFORE_TURN     = 5_deg;

// Controller
constexpr double CONTROLLER_DEADBAND = 0.1;

// Component speeds (in percent)
constexpr double DRIVE_SPEED = 1.0;
