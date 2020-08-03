#pragma once

#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QSpeed.hpp"

struct ChassisSpeeds {
    // +x is forward
    okapi::QSpeed vx;
    // +y is to the right
    okapi::QSpeed vy;
    // +omega is counter-clockwise
    okapi::QAngularSpeed omega;
};