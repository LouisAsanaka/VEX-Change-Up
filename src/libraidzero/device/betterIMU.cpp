#include "libraidzero/device/betterIMU.hpp"

BetterIMU::BetterIMU(std::uint8_t iport, IMUAxes iaxis) 
    : IMU{iport, iaxis} {} 

double BetterIMU::getRawAngle() {
    const double angle = readAngle();
    if (angle == PROS_ERR) {
        return PROS_ERR;
    }
    return angle - offset;
}

std::int32_t BetterIMU::reset(QAngle iangle) {
    offset = readAngle();
    if (offset == PROS_ERR) {
        return PROS_ERR;
    }
    offset += iangle.convert(degree);
    return 1;
}