#pragma once

#include "okapi/api/units/QAngle.hpp"
#include "okapi/impl/device/rotarysensor/IMU.hpp"

using namespace okapi::literals;

class BetterIMU : public okapi::IMU {
public:
    BetterIMU(std::uint8_t iport, okapi::IMUAxes iaxis = okapi::IMUAxes::z);

    double getRawAngle();
    std::int32_t reset(okapi::QAngle iangle = 0_deg);
};