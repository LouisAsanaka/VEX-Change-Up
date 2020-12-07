#include "libraidzero/odometry/threeEncoderImuOdometry.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <cmath>

ThreeEncoderImuOdometry::ThreeEncoderImuOdometry(
    const TimeUtil &itimeUtil,
    std::shared_ptr<ThreeEncoderImuXDriveModel> imodel,
    const ChassisScales &ichassisScales,
    std::shared_ptr<Logger> ilogger
) : logger{std::move(ilogger)},
    rate{itimeUtil.getRate()},
    timer{itimeUtil.getTimer()},
    model{std::move(imodel)},
    chassisScales{ichassisScales} {}

void ThreeEncoderImuOdometry::setScales(const ChassisScales &ichassisScales) {
    chassisScales = ichassisScales;
}

void ThreeEncoderImuOdometry::step() {
    const auto deltaT = timer->getDt();

    if (deltaT.getValue() != 0) {
        newTicks = model->getSensorVals();
        tickDiff = newTicks - lastTicks;
        lastTicks = newTicks;

        newHeading = model->getHeading();
        headingDiff = newHeading - lastHeading;
        lastHeading = newHeading;

        const auto newState = odomMathStep(tickDiff, headingDiff);

        state.x += newState.x;
        state.y += newState.y;
        state.theta += newState.theta;
    }
}

OdomState ThreeEncoderImuOdometry::odomMathStep(
    const std::valarray<std::int32_t> &itickDiff,
    const double iheadingDiff
) {
    if (itickDiff.size() < 3) {
        LOG_ERROR_S("ThreeEncoderImuOdometry: itickDiff did not have at least three elements.");
        return OdomState{};
    }

    for (auto &&elem : itickDiff) {
        if (std::abs(elem) > maximumTickDiff) {
            LOG_ERROR("ThreeEncoderImuOdometry: A tick diff (" + std::to_string(elem) +
                      ") was greater than the maximum allowable diff (" +
                      std::to_string(maximumTickDiff) + "). Skipping this odometry step.");
            return OdomState{};
        }
    }

    const double deltaL = itickDiff[0] / chassisScales.straight;
    const double deltaR = itickDiff[1] / chassisScales.straight;

    //double deltaTheta = (deltaL - deltaR) / chassisScales.wheelTrack.convert(meter);
    // TODO: Check if IMU odometry is viable
    double deltaTheta = iheadingDiff * okapi::pi / 180;
    double localOffX, localOffY;

    const auto deltaM = static_cast<const double>(
        itickDiff[2] / chassisScales.middle -
        ((deltaTheta / 2_pi) * 1_pi * chassisScales.middleWheelDistance.convert(meter) * 2));

    if (deltaL == deltaR) {
        localOffX = deltaM;
        localOffY = deltaR;
    } else {
        localOffX = 2 * std::sin(deltaTheta / 2) *
                                (deltaM / deltaTheta + chassisScales.middleWheelDistance.convert(meter) * 2);
        localOffY = 2 * std::sin(deltaTheta / 2) *
                                (deltaR / deltaTheta + chassisScales.wheelTrack.convert(meter) / 2);
    }

    double avgA = state.theta.convert(radian) + (deltaTheta / 2);

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

    return OdomState{dX * meter, dY * meter, deltaTheta * radian};
}

OdomState ThreeEncoderImuOdometry::getState(const StateMode &imode) const {
    if (imode == StateMode::FRAME_TRANSFORMATION) {
        return state;
    }
    return OdomState{state.y, state.x, state.theta};
}

void ThreeEncoderImuOdometry::setState(const OdomState &istate, const StateMode &imode) {
    LOG_DEBUG("State set to: " + istate.str());
    if (imode == StateMode::FRAME_TRANSFORMATION) {
        state = istate;
    } else {
        state = OdomState{istate.y, istate.x, istate.theta};
    }
}

std::shared_ptr<ThreeEncoderImuXDriveModel> ThreeEncoderImuOdometry::getModel() {
    return model;
}

ChassisScales ThreeEncoderImuOdometry::getScales() {
    return chassisScales;
}
