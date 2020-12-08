#pragma once

#include "okapi/api.hpp"

using namespace okapi;

class IOdomController {
public:
    enum class TurnType {
        LeftPivot, PointTurn, RightPivot
    };

    /**
     * Drives the robot for the specified distance and wait until the robot
     * reaches the point.
     * 
     * @param idistance The distance to drive for
     * @param itimeout The timeout for the movement
     */
    virtual void driveForDistance(QLength idistance, int itimeout = 0) = 0;

    /**
     * Drives the robot for the specified distance.
     * 
     * @param idistance The distance to drive for
     */
    virtual void driveForDistanceAsync(QLength idistance) = 0;

    /**
     * Drives the robot straight to a point in the odom frame.
     *
     * @param ipoint The target point to navigate to
     * @param ibackwards Whether to drive to the target point backwards
     * @param itimeout The timeout for the movement
     */
    virtual void driveToPoint(const Point& ipoint, bool ibackwards = false, int itimeout = 0) = 0;

    /**
     * Turn the robot for the specified angle and wait until the robot
     * reaches the angle.
     * 
     * @param iangle The angle to turn
     * @param iturnType The type of turn to make
     * @param itimeout The timeout for the movement
     */
    virtual void turnAngle(QAngle iangle, TurnType iturnType = TurnType::PointTurn, int itimeout = 0) = 0;

    /**
     * Turn the robot for the specified angle.
     * 
     * @param iangle The angle to turn
     * @param iturnType The type of turn to make
     */
    virtual void turnAngleAsync(QAngle iangle, TurnType iturnType = TurnType::PointTurn) = 0;

    /**
     * Turns the robot to the specified angle in the odom frame.
     *
     * @param iangle The global angle to turn to
     * @param iturnType The type of turn to make
     * @param itimeout The timeout for the movement
     */
    virtual void turnToAngle(QAngle iangle, TurnType iturnType = TurnType::PointTurn, int itimeout = 0) = 0;

    /**
     * Turns the robot to face a point in the odom frame.
     *
     * @param ipoint The target point to turn towards
     * @param itimeout The timeout for the movement
     */
    virtual void turnToPoint(const Point& ipoint, int itimeout = 0) = 0;

    /**
     * Sets the current odometry state.
     *
     * @param istate the new state
     */
    virtual void setState(OdomState istate) = 0;

    /**
     * Returns the current odometry state.
     *
     * @return the odometry state
     */
    virtual OdomState getState() = 0;

    /**
     * Returns whether the current movement has settled.
     *
     * @return whether the current movement has settled
     */
    virtual bool isSettled() = 0;

    enum class SettleResult {
        Settled, NotSettled, Timeout
    };

    /**
     * Blocks until the current movement is settled or timeouts.
     * 
     * @param itimeout the timeout period for the movement
     * 
     * @return whether the movement eventually settled or not
     */
    virtual SettleResult waitUntilSettled(int itimeout = 0) = 0;

    /**
     * Stops the chassis and disables all PID controllers.
     */
    virtual void stop() = 0;

    /**
     * Sets a new maximum voltage in mV [0-12000].
     *
     * @param imaxVoltage The new maximum voltage.
     */
    virtual void setMaxVoltage(int imaxVoltage) = 0;

    /**
     * @return The maximum voltage in mV [0-12000].
     */
    virtual int getMaxVoltage() const = 0;

    /**
     * @return The internal ChassisModel.
     */
    virtual std::shared_ptr<ChassisModel> getModel() = 0;

    /**
     * Gets the ChassisScales.
     */
    virtual ChassisScales getChassisScales() const = 0;

    /**
     * Gets the GearsetRatioPair.
     */
    virtual AbstractMotor::GearsetRatioPair getGearsetRatioPair() const = 0;

    /**
     * Returns the internal thread that executes movements.
     *
     * @return internal thread
     */
    virtual CrossplatformThread* getThread() const = 0;

    /**
     * Returns the internal thread that does odometry.
     *
     * @return internal odometry thread
     */
    virtual CrossplatformThread* getOdomThread() const = 0;

    /**
     * Starts the internal thread that executes movements.
     */
    virtual void startThread() = 0;

    /**
     * Starts the internal thread that does odometry.
     */
    virtual void startOdomThread() = 0;

    /**
     * Blocks the thread until the odometry task is started.
     */
    virtual void waitForOdomTask() = 0;
};