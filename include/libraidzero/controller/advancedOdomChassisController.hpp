/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "libraidzero/controller/advancedChassisController.hpp"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/chassis/model/skidSteerModel.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include <memory>

using namespace okapi;

class AdvancedOdomChassisController : public OdomChassisController {
    typedef AdvancedChassisController::TurnType TurnType;
public:
    /**
     * Odometry based chassis controller that moves using a separately constructed chassis controller.
     * Spins up a task at the default priority plus 1 for odometry when constructed.
     *
     * Moves the robot around in the odom frame. Instead of telling the robot to drive forward or
     * turn some amount, you instead tell it to drive to a specific point on the field or turn to
     * a specific angle, relative to its starting position.
     *
     * @param itimeUtil The TimeUtil.
     * @param iodometry The odometry to read state estimates from.
     * @param icontroller The chassis controller to delegate to.
     * @param imode The new default StateMode used to interpret target points and query the Odometry
     * state.
     * @param imoveThreshold minimum length movement (smaller movements will be skipped)
     * @param iturnThreshold minimum angle turn (smaller turns will be skipped)
     * @param ilogger The logger this instance will log to.
     */
    AdvancedOdomChassisController(const TimeUtil &itimeUtil,
                                  std::shared_ptr<Odometry> iodometry,
                                  std::shared_ptr<ChassisController> icontroller,
                                  const StateMode &imode = StateMode::FRAME_TRANSFORMATION,
                                  QLength imoveThreshold = 0_mm,
                                  QAngle iturnThreshold = 0_deg,
                                  std::shared_ptr<Logger> ilogger = Logger::getDefaultLogger());

    AdvancedOdomChassisController(const AdvancedOdomChassisController &) = delete;
    AdvancedOdomChassisController(AdvancedOdomChassisController &&other) = delete;
    AdvancedOdomChassisController &operator=(const AdvancedOdomChassisController &other) = delete;
    AdvancedOdomChassisController &operator=(AdvancedOdomChassisController &&other) = delete;

    /**
     * Drives the robot straight to a point in the odom frame.
     *
     * @param ipoint The target point to navigate to.
     * @param ibackwards Whether to drive to the target point backwards.
     * @param ioffset An offset from the target point in the direction pointing towards the robot. The
     * robot will stop this far away from the target point.
     */
    void driveToPoint(const Point &ipoint,
                      bool ibackwards = false,
                      const QLength &ioffset = 0_mm) override;

    /**
     * Turns the robot to face a point in the odom frame.
     *
     * @param ipoint The target point to turn to face.
     */
    void turnToPoint(const Point &ipoint) override;

    /**
     * @return The internal ChassisController.
     */
    std::shared_ptr<ChassisController> getChassisController();

    /**
     * @return The internal ChassisController.
     */
    ChassisController &chassisController();

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void turnToAngle(const QAngle &iangle) override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void moveDistance(QLength itarget) override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void moveRaw(double itarget) override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void moveDistanceAsync(QLength itarget) override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void moveRawAsync(double itarget) override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void turnAngle(QAngle idegTarget) override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void turnAngle(QAngle idegTarget, TurnType iturnType);

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void turnRaw(double idegTarget) override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void turnRaw(double idegTarget, TurnType iturnType);

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void turnAngleAsync(QAngle idegTarget) override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void turnAngleAsync(QAngle idegTarget, TurnType iturnType);

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void turnRawAsync(double idegTarget) override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void turnRawAsync(double idegTarget, TurnType iturnType);

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void setTurnsMirrored(bool ishouldMirror) override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    bool isSettled() override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void waitUntilSettled() override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void stop() override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    void setMaxVelocity(double imaxVelocity) override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    double getMaxVelocity() const override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    ChassisScales getChassisScales() const override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    AbstractMotor::GearsetRatioPair getGearsetRatioPair() const override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    std::shared_ptr<ChassisModel> getModel() override;

    /**
     * This delegates to the input AdvancedChassisController.
     */
    ChassisModel &model() override;

protected:
    std::shared_ptr<Logger> logger;
    std::shared_ptr<AdvancedChassisController> controller;

    void waitForOdomTask();
};
