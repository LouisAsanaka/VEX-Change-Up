#pragma once

#include "main.h"

struct ChassisSpeeds {
    // +x is forward
    QSpeed vx;
    // +y is to the right
    QSpeed vy;
    // +omega is counter-clockwise
    QAngularSpeed omega;
};