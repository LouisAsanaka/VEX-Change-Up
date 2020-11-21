#pragma once

#include "okapi/api/units/QAngle.hpp"

void reset(okapi::QAngle initialAngle);
void backout(int milliseconds);

void rightSide1(bool shouldReset = true, bool shouldBackOut = true);
void rightSide2(bool shouldReset = true, bool shouldBackOut = true);
void rightSide3(bool shouldReset = true);

void autonomous();
