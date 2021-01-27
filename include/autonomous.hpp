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

enum class StartingSide {
    Left, Right
};

void runSide1(StartingSide side, bool shouldReset = true, bool shouldBackOut = true);
void runSide2(StartingSide side, bool shouldReset = true, bool shouldBackOut = true);
void runSide3(StartingSide side, bool shouldReset = true, bool shouldBackOut = true);
void runSideCenter(StartingSide side, bool shouldReset = true, bool shouldBackOut = true);
void runWashCorner(StartingSide side, bool shouldReset = true, bool shouldBackOut = true);
void runLeftSideMidCenter(bool shouldReset = true, bool shouldBackOut = true);

void autonomous();
