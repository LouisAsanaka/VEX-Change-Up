#include "kinematics/driveWheelSpeeds.hpp"
#include "main.h"
#include <cmath>

void DriveWheelSpeeds::normalize(QSpeed maxSpeed) {
    auto realMaxSpeed = std::max(left.abs().convert(mps), right.abs().convert(mps));
    auto specifiedMaxSpeed = maxSpeed.convert(mps);

    if (realMaxSpeed > specifiedMaxSpeed) {
        left = (left.convert(mps) / realMaxSpeed * specifiedMaxSpeed) * mps;
        right = (right.convert(mps) / realMaxSpeed * specifiedMaxSpeed) * mps;
    }
}