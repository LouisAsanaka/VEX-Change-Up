#pragma once

#include "main.h"

struct DriveWheelSpeeds {

    QSpeed left = 0_mps;
    QSpeed right = 0_mps;

    void normalize(QSpeed maxSpeed);
};