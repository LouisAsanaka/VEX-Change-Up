#pragma once

#include "okapi/api/units/QSpeed.hpp"

using namespace okapi::literals;

struct DriveWheelSpeeds {

    okapi::QSpeed left = 0_mps;
    okapi::QSpeed right = 0_mps;

    void normalize(okapi::QSpeed maxSpeed);
};