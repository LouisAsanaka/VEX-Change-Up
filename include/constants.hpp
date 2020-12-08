#pragma once

// Robot dimensions
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QTime.hpp"

using namespace okapi::literals;

#define RUN_WITHOUT_ROBOT 1

constexpr int OPCONTROL_DURATION = 105 * 1000;
constexpr int TIME_TILL_15_SECONDS = OPCONTROL_DURATION - 15 * 1000;

constexpr okapi::QLength WHEEL_TRACK                  = 12.9_in; //12.63
constexpr okapi::QLength TRACKING_WHEEL_DIAMETER      = 2.80_in;
constexpr okapi::QLength TRACKING_BACK_WHEEL_DISTANCE = 5.7_in;

// Drive profiling constants
constexpr double DRIVE_MAX_VEL   = 0.5;
constexpr double DRIVE_MAX_ACCEL = 0.5;
constexpr double DRIVE_MAX_JERK  = 8.0;

// Drivetrain settling tolerances
constexpr double DRIVE_ENCODER_TARGET_ERROR       = 40;
constexpr double DRIVE_ENCODER_TARGET_DERIV       = 5;
constexpr okapi::QTime DRIVE_ENCODER_TARGET_TIME  = 200_ms;

constexpr double STRAFING_DIST_TARGET_ERROR       = 0.02;
constexpr double STRAFING_DIST_TARGET_DERIV       = 5;
constexpr okapi::QTime STRAFING_DIST_TARGET_TIME  = 150_ms;

constexpr double STRAFING_ANGLE_TARGET_ERROR      = 0.1;
constexpr double STRAFING_ANGLE_TARGET_DERIV      = 5;
constexpr okapi::QTime STRAFING_ANGLE_TARGET_TIME = 150_ms;

constexpr okapi::QLength DISTANCE_BEFORE_MOVE = 5_cm;
constexpr okapi::QAngle ANGLE_BEFORE_TURN     = 5_deg;

// Drivetrain PID constants
constexpr double DISTANCE_KP = 0.0035;
constexpr double DISTANCE_KI = 0.0;
constexpr double DISTANCE_KD = 0.00007;

constexpr double ANGLE_KP = 0.002;
constexpr double ANGLE_KI = 0.0;
constexpr double ANGLE_KD = 0.0;

constexpr double TURN_KP = 0.0045;
constexpr double TURN_KI = 0.001;
constexpr double TURN_KD = 0.00009;

constexpr double STRAFE_DISTANCE_KP = 3.3;
constexpr double STRAFE_DISTANCE_KI = 0.0;
constexpr double STRAFE_DISTANCE_KD = 0.004;

constexpr double STRAFE_ANGLE_KP = 2.1;
constexpr double STRAFE_ANGLE_KI = 0.0;
constexpr double STRAFE_ANGLE_KD = 0.03;

constexpr double STRAFE_SLEW_RATE = 0.10; // m / ms

// Controller
constexpr double CONTROLLER_DEADBAND = 0.1;

// Component speeds (in percent)
constexpr double DRIVE_SPEED = 1.0;
