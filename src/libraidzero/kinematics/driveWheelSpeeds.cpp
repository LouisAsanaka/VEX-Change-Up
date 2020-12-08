#include "libraidzero/kinematics/driveWheelSpeeds.hpp"

#include <cmath>

void DriveWheelSpeeds::normalize(okapi::QSpeed maxSpeed) {
    auto realMaxSpeed = std::max(left.abs().convert(okapi::mps), right.abs().convert(okapi::mps));
    auto specifiedMaxSpeed = maxSpeed.convert(okapi::mps);

    if (realMaxSpeed > specifiedMaxSpeed) {
        left = (left.convert(okapi::mps) / realMaxSpeed * specifiedMaxSpeed) * okapi::mps;
        right = (right.convert(okapi::mps) / realMaxSpeed * specifiedMaxSpeed) * okapi::mps;
    }
}