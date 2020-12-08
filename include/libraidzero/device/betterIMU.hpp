#pragma once

#include "okapi/api.hpp"

using namespace okapi;

class BetterIMU : public IMU {
public:
    BetterIMU(std::uint8_t iport, IMUAxes iaxis = IMUAxes::z);

    double getRawAngle();
    std::int32_t reset(QAngle iangle = 0_deg);
};