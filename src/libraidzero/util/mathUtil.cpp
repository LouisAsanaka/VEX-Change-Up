#include "libraidzero/util/mathUtil.hpp"

#include "okapi/api/util/mathUtil.hpp"

#include <cmath>

double constrainAnglePi(double radians) {
    return radians - 2 * okapi::pi * std::floor((radians + okapi::pi) / (2 * okapi::pi));
}

double constrainAngle2Pi(double radians) {
    return radians - 2 * okapi::pi * std::floor(radians / (2 * okapi::pi));
}

double constrainAngle180(double degrees) {
    return degrees - 360 * std::floor((degrees + 180) / 360);
}

double constrainAngle360(double degrees) {
    return degrees - 360 * std::floor(degrees / 360);
}
