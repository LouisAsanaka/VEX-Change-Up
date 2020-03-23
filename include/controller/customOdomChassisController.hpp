#pragma once

#include "main.h"
#include <memory>

class CustomOdomChassisController : public OdomChassisController {
public:
    /**
     * Odometry based chassis controller that moves using the V5 motor's integrated control. Spins up
     * a task at the default priority plus 1 for odometry when constructed.
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
    CustomOdomChassisController(const TimeUtil &itimeUtil,
                                std::unique_ptr<Odometry> iodometry,
                                std::shared_ptr<ChassisController> icontroller,
                                const StateMode &imode = StateMode::FRAME_TRANSFORMATION,
                                QLength imoveThreshold = 0_mm,
                                QAngle iturnThreshold = 0_deg,
                                std::shared_ptr<Logger> ilogger = Logger::getDefaultLogger());

    CustomOdomChassisController(const CustomOdomChassisController &) = delete;
    CustomOdomChassisController(CustomOdomChassisController &&other) = delete;
    CustomOdomChassisController &operator=(const CustomOdomChassisController &other) = delete;
    CustomOdomChassisController &operator=(CustomOdomChassisController &&other) = delete;

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
     * Resets the odometry state.
     */
    void resetState();

    /**
     * @return The internal ChassisController.
     */
    std::shared_ptr<ChassisController> getChassisController();

    /**
     * @return The internal ChassisController.
     */
    ChassisController &chassisController();

    /**
     * This delegates to the input ChassisController.
     */
    void turnToAngle(const QAngle &iangle) override;

    /**
     * This delegates to the input ChassisController.
     */
    void moveDistance(QLength itarget) override;

    /**
     * This delegates to the input ChassisController.
     */
    void moveDistance(double itarget) override;

    /**
     * This delegates to the input ChassisController.
     */
    void moveDistanceAsync(QLength itarget) override;

    /**
     * This delegates to the input ChassisController.
     */
    void moveDistanceAsync(double itarget) override;

    /**
     * This delegates to the input ChassisController.
     */
    void turnAngle(QAngle idegTarget) override;

    /**
     * This delegates to the input ChassisController.
     */
    void turnAngle(double idegTarget) override;

    /**
     * This delegates to the input ChassisController.
     */
    void turnAngleAsync(QAngle idegTarget) override;

    /**
     * This delegates to the input ChassisController.
     */
    void turnAngleAsync(double idegTarget) override;

    /**
     * This delegates to the input ChassisController.
     */
    void setTurnsMirrored(bool ishouldMirror) override;

    /**
     * This delegates to the input ChassisController.
     */
    bool isSettled() override;

    /**
     * This delegates to the input ChassisController.
     */
    void waitUntilSettled() override;

    /**
     * This delegates to the input ChassisController.
     */
    void stop() override;

    /**
     * This delegates to the input ChassisController.
     */
    ChassisScales getChassisScales() const override;

    /**
     * This delegates to the input ChassisController.
     */
    AbstractMotor::GearsetRatioPair getGearsetRatioPair() const override;

    /**
     * This delegates to the input ChassisController.
     */
    std::shared_ptr<ChassisModel> getModel() override;

    /**
     * This delegates to the input ChassisController.
     */
    ChassisModel &model() override;

protected:
    std::shared_ptr<Logger> logger;
    std::shared_ptr<ChassisController> controller;
};
