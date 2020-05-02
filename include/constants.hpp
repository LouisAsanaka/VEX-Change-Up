#pragma once

#include "main.h"

// Robot dimensions
constexpr QLength DRIVE_WHEEL_DIAMETER         = 4_in;
constexpr QLength DRIVE_WHEEL_TRACK            = 14.5_in;
constexpr QLength TRACKING_WHEEL_DIAMETER      = 2.75_in;
constexpr QLength TRACKING_BACK_WHEEL_DISTANCE = 13.2_in;

// Drive profiling constants
constexpr double DRIVE_MAX_VEL   = 0.5;
constexpr double DRIVE_MAX_ACCEL = 0.5;
constexpr double DRIVE_MAX_JERK  = 8.0;

// Controller
constexpr double CONTROLLER_DEADBAND = 0.1;

// Component speeds (in percent)
constexpr double DRIVE_SPEED = 0.90;
