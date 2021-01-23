#pragma once

#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

using namespace okapi::literals;

void reset(okapi::QLength x = 0_m, okapi::QLength y = 0_m, 
           okapi::QAngle initialAngle = 0_deg, bool currentAngle = false);
void backout(int milliseconds);
void backupFromGoal();
void releaseComponents();
void scoreOneBall();

void rightSide1(bool shouldReset = true, bool shouldBackOut = true);
void rightSide2(bool shouldReset = true, bool shouldBackOut = true);
void rightSide3(bool shouldReset = true, bool shouldBackOut = true);
void rightSideCenter(bool shouldReset = true, bool shouldBackOut = true);

void autonomous();
