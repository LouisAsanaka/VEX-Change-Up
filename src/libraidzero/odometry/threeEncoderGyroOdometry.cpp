/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "libraidzero/odometry/threeEncoderGyroOdometry.hpp"
#include "libraidzero/chassis/model/threeEncoderGyroXDriveModel.hpp"
#include "okapi/api/odometry/threeEncoderOdometry.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <cmath>

using namespace okapi::literals;

ThreeEncoderGyroOdometry::ThreeEncoderGyroOdometry(const okapi::TimeUtil &itimeUtil,
                                                   const std::shared_ptr<okapi::ReadOnlyChassisModel> &imodel,
                                                   const okapi::ChassisScales &ichassisScales,
                                                   const std::shared_ptr<okapi::Logger> &logger)
    : ThreeEncoderOdometry(itimeUtil, imodel, ichassisScales, logger) {
    if (ichassisScales.middle == 0) {
        std::string msg = "ThreeEncoderGyroOdometry: Middle scale cannot be zero.";
        LOG_ERROR(msg);
        throw std::invalid_argument(msg);
    }
}

okapi::OdomState ThreeEncoderGyroOdometry::odomMathStep(const std::valarray<std::int32_t> &itickDiff,
                                                        const okapi::QTime &ideltaT) {
    if (itickDiff.size() < 4) {
        LOG_ERROR_S("ThreeEncoderGyroOdometry: itickDiff did not have at least four elements.");
        return okapi::OdomState{};
    }

    for (auto &&elem : itickDiff) {
        if (std::abs(elem) > maximumTickDiff) {
            LOG_ERROR("ThreeEncoderGyroOdometry: A tick diff (" + std::to_string(elem) +
                      ") was greater than the maximum allowable diff (" +
                      std::to_string(maximumTickDiff) + "). Skipping this odometry step.");
            return okapi::OdomState{};
        }
    }

    const double deltaL = itickDiff[0] / chassisScales.straight;
    const double deltaR = itickDiff[1] / chassisScales.straight;

    double deltaTheta = (deltaL - deltaR) / chassisScales.wheelTrack.convert(okapi::meter);
    //double deltaTheta = itickDiff[3] / GYRO_RESOLUTION * okapi::pi / 180;
    //std::cout << "Delta theta: " << deltaTheta << std::endl;
    double localOffX, localOffY;

    const auto deltaM = static_cast<const double>(
        itickDiff[2] / chassisScales.middle -
        ((deltaTheta / 2_pi) * 1_pi * chassisScales.middleWheelDistance.convert(okapi::meter) * 2));

    if (deltaL == deltaR) {
        localOffX = deltaM;
        localOffY = deltaR;
    } else {
        localOffX = 2 * std::sin(deltaTheta / 2) *
                                (deltaM / deltaTheta + chassisScales.middleWheelDistance.convert(okapi::meter) * 2);
        localOffY = 2 * std::sin(deltaTheta / 2) *
                                (deltaR / deltaTheta + chassisScales.wheelTrack.convert(okapi::meter) / 2);
    }

    double avgA = state.theta.convert(okapi::radian) + (deltaTheta / 2);

    double polarR = std::sqrt((localOffX * localOffX) + (localOffY * localOffY));
    double polarA = std::atan2(localOffY, localOffX) - avgA;

    double dX = std::sin(polarA) * polarR;
    double dY = std::cos(polarA) * polarR;

    if (isnan(dX)) {
        dX = 0;
    }

    if (isnan(dY)) {
        dY = 0;
    }

    if (isnan(deltaTheta)) {
        deltaTheta = 0;
    }

    return okapi::OdomState{dX * okapi::meter, dY * okapi::meter, deltaTheta * okapi::radian};
}
