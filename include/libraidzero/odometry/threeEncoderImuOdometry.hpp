#pragma once

#include "libraidzero/chassis/model/threeEncoderImuXDriveModel.hpp"

#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/odometry/odomState.hpp"
#include "okapi/api/odometry/stateMode.hpp"
#include "okapi/api/util/timeUtil.hpp"

#include <functional>

using namespace okapi;

class ThreeEncoderImuOdometry {
public:
    /**
     * Odometry. Tracks the movement of the robot and estimates its position in coordinates
     * relative to the start (assumed to be (0, 0)).
     *
     * @param itimeUtil The TimeUtil.
     * @param imodel The chassis model for reading sensors.
     * @param ichassisScales See ChassisScales docs (the middle wheel scale is the third member)
     * @param iwheelVelDelta The maximum delta between wheel velocities to consider the robot as
     * driving straight.
     * @param ilogger The logger this instance will log to.
     */
    ThreeEncoderImuOdometry(
        const TimeUtil &itimeUtil,
        std::shared_ptr<ThreeEncoderImuXDriveModel> imodel,
        const ChassisScales &ichassisScales,
        std::shared_ptr<Logger> ilogger = Logger::getDefaultLogger()
    );
    /**
     * Sets the drive and turn scales.
     */
    void setScales(const ChassisScales &ichassisScales);

    /**
     * Do one odometry step.
     */
    void step();

    /**
     * Returns the current state.
     *
     * @param imode The mode to return the state in.
     * @return The current state in the given format.
     */
    OdomState getState(const StateMode &imode = StateMode::FRAME_TRANSFORMATION) const;

    /**
     * Sets a new state to be the current state.
     *
     * @param istate The new state in the given format.
     * @param imode The mode to treat the input state as.
     */
    void setState(const OdomState &istate,
                  const StateMode &imode = StateMode::FRAME_TRANSFORMATION);

    /**
     * @return The internal ChassisModel.
     */
    std::shared_ptr<ThreeEncoderImuXDriveModel> getModel();

    /**
     * @return The internal ChassisScales.
     */
    ChassisScales getScales();

protected:
    std::shared_ptr<Logger> logger;
    std::unique_ptr<AbstractRate> rate;
    std::unique_ptr<AbstractTimer> timer;
    std::shared_ptr<ThreeEncoderImuXDriveModel> model;
    ChassisScales chassisScales;
    OdomState state;
    std::valarray<std::int32_t> newTicks{0, 0, 0}, tickDiff{0, 0, 0}, lastTicks{0, 0, 0};
    double newHeading{0.0}, headingDiff{0.0}, lastHeading{0.0};
    const std::int32_t maximumTickDiff{1000};

    /**
     * Does the math, side-effect free, for one odom step.
     *
     * @param itickDiff The tick difference from the previous step to this step.
     * @param iheadingDiff The heading difference from the previous step to this step.
     * @return The newly computed OdomState.
     */
    virtual OdomState odomMathStep(
        const std::valarray<std::int32_t> &itickDiff,
        const double iheadingDiff
    );
};
