#pragma once

#include "libraidzero/action/asyncAction.hpp"
#include "libraidzero/chassis/controller/iodomController.hpp"
#include "libraidzero/chassis/model/threeEncoderImuXDriveModel.hpp"
#include "libraidzero/filter/slewRateLimiter.hpp"
#include "libraidzero/geometry/pose2d.hpp"
#include "libraidzero/odometry/threeEncoderImuOdometry.hpp"
#include "libraidzero/util/taskWrapper.hpp"

#include "okapi/api.hpp"

#define makeSettlableLoop(settleFunction, itimeout, iactions, iunitsError, loopedBody) \
    uint32_t start = pros::millis();                            \
    uint32_t timeElapsed = pros::millis() - start;              \
    bool timeLeft = (timeElapsed < itimeout);                   \
    bool settled = settleFunction();                            \
    auto rate = timeUtil.getRate();                             \
    while (!settled && timeLeft && task->notifyTake(0) == 0U) { \
        loopedBody                                              \
        timeElapsed = pros::millis() - start;                   \
        executeActions(iunitsError, timeElapsed, iactions);     \
        settled = settleFunction();                             \
        timeLeft = (timeElapsed < itimeout);                    \
        rate->delayUntil(10_ms);                                \
    }

#define parseTimeout(timeoutVar)  \
    if (timeoutVar == 0) {        \
        timeoutVar = ~timeoutVar; \
    }

using namespace okapi;

class XOdomController : public TaskWrapper {
public:
    enum class TurnType {
        LeftPivot, PointTurn, RightPivot
    };

    /**
     * X-drive chassis control using odometry.
     *
     * @param itimeUtil The TimeUtil.
     * @param imodel The XDriveModel used to read from sensors/write to motors.
     * @param iodometry The odometry to read state estimates from.
     * @param idistancePid The PID controller that controls chassis distance for driving
     * straight.
     * @param ianglePid The PID controller that controls chassis angle for driving straight.
     * @param iturnPid The PID controller that controls chassis angle for turning.
     * @param istrafeDistancePid The PID controller that controls chassis distance while strafing.
     * @param istrafeAnglePid The PID controller that controls chassis angle while strafing.
     * @param igearset The internal gearset and external ratio used on the drive motors.
     * @param iscales The ChassisScales.
     * @param idistanceThreshold minimum length movement (smaller movements will be skipped)
     * @param iturnThreshold minimum angle turn (smaller turns will be skipped)
     * @param ilogger The logger this instance will log to.
     */
    XOdomController(
        TimeUtil itimeUtil,
        std::shared_ptr<ThreeEncoderImuXDriveModel> imodel,
        std::shared_ptr<ThreeEncoderImuOdometry> iodometry,
        std::unique_ptr<IterativePosPIDController> idistancePid,
        std::unique_ptr<IterativePosPIDController> ianglePid,
        std::unique_ptr<IterativePosPIDController> iturnPid,
        std::unique_ptr<IterativePosPIDController> istrafeDistancePid,
        std::unique_ptr<IterativePosPIDController> istrafeAnglePid,
        std::unique_ptr<SlewRateLimiter> islewRate,
        const AbstractMotor::GearsetRatioPair& igearset,
        const ChassisScales& iscales,
        QLength idistanceThreshold = 0_mm,
        QAngle iturnThreshold = 0_deg,
        std::shared_ptr<Logger> ilogger = Logger::getDefaultLogger());

    XOdomController(const XOdomController&) = delete;
    XOdomController(XOdomController&& other) = delete;
    XOdomController &operator=(const XOdomController& other) = delete;
    XOdomController &operator=(XOdomController&& other) = delete;
    ~XOdomController() override;

    /**
     * Drives the robot for the specified distance and wait until the robot
     * reaches the point.
     * 
     * @param idistance The distance to drive for
     * @param itimeout The timeout for the movement
     */
    void driveForDistance(QLength idistance, int itimeout = 0, 
        std::vector<AsyncAction> iactions = {});

    /**
     * Drives the robot straight to a point in the odom frame.
     *
     * @param ipoint The target point to navigate to
     * @param ibackwards Whether to drive to the target point backwards
     * @param itimeout The timeout for the movement
     */
    void driveToPoint(const Point& ipoint, bool ibackwards = false, int itimeout = 0, 
        std::vector<AsyncAction> iactions = {});

    /**
     * Turn the robot for the specified angle and wait until the robot
     * reaches the angle.
     * 
     * @param iangle The angle to turn
     * @param iturnType The type of turn to make
     * @param itimeout The timeout for the movement
     */
    void turnAngle(QAngle iangle, TurnType iturnType = TurnType::PointTurn, int itimeout = 0, 
        std::vector<AsyncAction> iactions = {});

    /**
     * Turns the robot to the specified angle in the odom frame.
     *
     * @param iangle The global angle to turn to
     * @param iturnType The type of turn to make
     * @param itimeout The timeout for the movement
     */
    void turnToAngle(QAngle iangle, TurnType iturnType = TurnType::PointTurn, int itimeout = 0, 
        std::vector<AsyncAction> iactions = {});

    /**
     * Turns the robot to face a point in the odom frame.
     *
     * @param ipoint The target point to turn towards
     * @param itimeout The timeout for the movement
     */
    void turnToPoint(const Point& ipoint, int itimeout = 0, 
        std::vector<AsyncAction> iactions = {});

    /**
     * Makes the robot strafe from the current pose to the target point, while
     * maintaining heading.
     *
     * @param ipoint The target point
     */
    void strafeToPoint(const Point& ipoint, int itimeout = 0,
        std::vector<AsyncAction> iactions = {});

    /**
     * Makes the robot strafe from the current pose to the target one.
     *
     * @param ipose The target pose
     */
    void strafeToPose(const Pose2d& ipose, int itimeout = 0,
        std::vector<AsyncAction> iactions = {});

    /**
     * Executes the actions depending on their conditions.
     * 
     * @param iunitsError The units error (distance / angle)
     * @param itimeFromStart The time elapsed since executing the movement
     * @param iactions The list of actions to execute
     */
    void executeActions(double iunitsError, int itimeFromStart, 
        std::vector<AsyncAction>& iactions);

    /**
     * Sets the current odometry state.
     *
     * @param istate the new state
     */
    void setState(OdomState istate);

    void setPose(const Pose2d& ipose);

    /**
     * Returns the current odometry state.
     *
     * @return the odometry state
     */
    OdomState getState();

    /**
     * Stops the chassis and disables all PID controllers.
     */
    void stop();

    /**
     * Sets a new maximum voltage in mV [0-12000].
     *
     * @param imaxVoltage The new maximum voltage.
     */
    void setMaxVoltage(int imaxVoltage);

    /**
     * @return The maximum voltage in mV [0-12000].
     */
    int getMaxVoltage() const;

    /**
     * @return The internal ChassisModel.
     */
    std::shared_ptr<ChassisModel> getModel();

    /**
     * Gets the ChassisScales.
     */
    ChassisScales getChassisScales() const;

    /**
     * Gets the GearsetRatioPair.
     */
    AbstractMotor::GearsetRatioPair getGearsetRatioPair() const;

    /**
     * Blocks the thread until the odometry task is started.
     */
    void waitForOdomTask();
protected:
    std::shared_ptr<ThreeEncoderImuXDriveModel> model {nullptr};
    std::shared_ptr<ThreeEncoderImuOdometry> odometry {nullptr};
    std::unique_ptr<IterativePosPIDController> distancePid {nullptr};
    std::unique_ptr<IterativePosPIDController> anglePid {nullptr};
    std::unique_ptr<IterativePosPIDController> turnPid {nullptr};
    std::unique_ptr<IterativePosPIDController> strafeDistancePid {nullptr};
    std::unique_ptr<IterativePosPIDController> strafeAnglePid {nullptr};
    std::unique_ptr<SlewRateLimiter> slewRate {nullptr};

    AbstractMotor::GearsetRatioPair gearsetRatioPair;
    ChassisScales scales;

    QLength distanceThreshold;
    QAngle turnThreshold;

    std::shared_ptr<Logger> logger;
    TimeUtil timeUtil;

    std::atomic_bool odomTaskRunning {false};
    std::atomic_bool dtorCalled {false};
    
    /**
     * Does one iteration of strafing to the target translation.
     * 
     * @param itargetTranslation the translation to strafe to
     * @param idistanceError the absolute distance error to write to
     */
    void updateStrafeToPose(const Translation2d& itargetTranslation, double& idistanceError);
    
    /**
     * Returns whether the drive has settled in distance mode.
     * 
     * @return is drive settled
     */
    bool isDistanceSettled();

    /**
     * Returns whether the drive has settled in angle mode.
     * 
     * @return is drive settled
     */
    bool isAngleSettled();

    /**
     * Returns whether the drive has settled in strafe mode.
     * 
     * @return is drive settled
     */
    bool isStrafeSettled();

    /**
     * Stops all the controllers and the ChassisModel.
     */
    void stopAfterSettled();

    /**
     * Runs odometry in the background.
     */
    void loop() override;
};
