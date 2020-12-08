#pragma once

#include "libraidzero/chassis/controller/iodomController.hpp"

#include "okapi/api.hpp"

using namespace okapi;

class OdomController : public IOdomController {
public:
    /**
     * Chassis control using odometry.
     *
     * @param itimeUtil The TimeUtil.
     * @param imodel The ChassisModel used to read from sensors/write to motors.
     * @param iodometry The odometry to read state estimates from.
     * @param idistancePid The PID controller that controls chassis distance for driving
     * straight.
     * @param ianglePid The PID controller that controls chassis angle for driving straight.
     * @param iturnPid The PID controller that controls chassis angle for turning.
     * @param igearset The internal gearset and external ratio used on the drive motors.
     * @param iscales The ChassisScales.
     * @param idistanceThreshold minimum length movement (smaller movements will be skipped)
     * @param iturnThreshold minimum angle turn (smaller turns will be skipped)
     * @param ilogger The logger this instance will log to.
     */
    OdomController(
        TimeUtil itimeUtil,
        std::shared_ptr<ChassisModel> imodel,
        std::shared_ptr<Odometry> iodometry,
        std::unique_ptr<IterativePosPIDController> idistancePid,
        std::unique_ptr<IterativePosPIDController> ianglePid,
        std::unique_ptr<IterativePosPIDController> iturnPid,
        const AbstractMotor::GearsetRatioPair& igearset,
        const ChassisScales& iscales,
        QLength idistanceThreshold = 0_mm,
        QAngle iturnThreshold = 0_deg,
        std::shared_ptr<Logger> ilogger = Logger::getDefaultLogger());

    OdomController(const OdomController&) = delete;
    OdomController(OdomController&& other) = delete;
    OdomController &operator=(const OdomController& other) = delete;
    OdomController &operator=(OdomController&& other) = delete;
    ~OdomController();

    /**
     * Drives the robot for the specified distance and wait until the robot
     * reaches the point.
     * 
     * @param idistance The distance to drive for
     * @param itimeout The timeout for the movement
     */
    void driveForDistance(QLength idistance, int itimeout = 0) override;

    /**
     * Drives the robot for the specified distance.
     * 
     * @param idistance The distance to drive for
     */
    void driveForDistanceAsync(QLength idistance) override;

    /**
     * Drives the robot straight to a point in the odom frame.
     *
     * @param ipoint The target point to navigate to
     * @param ibackwards Whether to drive to the target point backwards
     * @param itimeout The timeout for the movement
     */
    void driveToPoint(const Point& ipoint, bool ibackwards = false, int itimeout = 0) override;

    /**
     * Turn the robot for the specified angle and wait until the robot
     * reaches the angle.
     * 
     * @param iangle The angle to turn
     * @param iturnType The type of turn to make
     * @param itimeout The timeout for the movement
     */
    void turnAngle(QAngle iangle, TurnType iturnType = TurnType::PointTurn, int itimeout = 0) override;

    /**
     * Turn the robot for the specified angle.
     * 
     * @param iangle The angle to turn
     * @param iturnType The type of turn to make
     */
    void turnAngleAsync(QAngle iangle, TurnType iturnType = TurnType::PointTurn) override;

    /**
     * Turns the robot to the specified angle in the odom frame.
     *
     * @param iangle The global angle to turn to
     * @param iturnType The type of turn to make
     * @param itimeout The timeout for the movement
     */
    void turnToAngle(QAngle iangle, TurnType iturnType = TurnType::PointTurn, int itimeout = 0) override;

    /**
     * Turns the robot to face a point in the odom frame.
     *
     * @param ipoint The target point to turn towards
     * @param itimeout The timeout for the movement
     */
    void turnToPoint(const Point& ipoint, int itimeout = 0) override;
    
    /**
     * Sets the current odometry state.
     *
     * @param istate the new state
     */
    void setState(OdomState istate) override;

    /**
     * Returns the current odometry state.
     *
     * @return the odometry state
     */
    OdomState getState() override;

    /**
     * Returns whether the current movement has settled.
     *
     * @return whether the current movement has settled
     */
    bool isSettled() override;

    /**
     * Blocks until the current movement is settled or timeouts.
     * 
     * @param itimeout the timeout period for the movement
     * 
     * @return whether the movement eventually settled or not
     */
    SettleResult waitUntilSettled(int itimeout = 0) override;

    /**
     * Stops the chassis and disables all PID controllers.
     */
    void stop() override;

    /**
     * Sets a new maximum voltage in mV [0-12000].
     *
     * @param imaxVoltage The new maximum voltage.
     */
    void setMaxVoltage(int imaxVoltage) override;

    /**
     * @return The maximum voltage in mV [0-12000].
     */
    int getMaxVoltage() const override;

    /**
     * @return The internal ChassisModel.
     */
    std::shared_ptr<ChassisModel> getModel() override;

    /**
     * Gets the ChassisScales.
     */
    ChassisScales getChassisScales() const override;

    /**
     * Gets the GearsetRatioPair.
     */
    AbstractMotor::GearsetRatioPair getGearsetRatioPair() const override;

    /**
     * Returns the internal thread that executes movements.
     *
     * @return internal thread
     */
    CrossplatformThread* getThread() const override;

    /**
     * Returns the internal thread that does odometry.
     *
     * @return internal odometry thread
     */
    CrossplatformThread* getOdomThread() const override;

    /**
     * Starts the internal thread that executes movements.
     */
    void startThread() override;

    /**
     * Starts the internal thread that does odometry.
     */
    void startOdomThread() override;

    /**
     * Blocks the thread until the odometry task is started.
     */
    void waitForOdomTask() override;
protected:
    std::shared_ptr<ChassisModel> model {nullptr};
    std::shared_ptr<Odometry> odometry {nullptr};
    std::unique_ptr<IterativePosPIDController> distancePid {nullptr};
    std::unique_ptr<IterativePosPIDController> anglePid {nullptr};
    std::unique_ptr<IterativePosPIDController> turnPid {nullptr};

    AbstractMotor::GearsetRatioPair gearsetRatioPair;
    ChassisScales scales;

    QLength distanceThreshold;
    QAngle turnThreshold;

    std::shared_ptr<Logger> logger;
    TimeUtil timeUtil;

    TurnType turnType = TurnType::PointTurn;

    enum class ControlMode {
        Distance, Angle, None
    };
    ControlMode mode = ControlMode::None;

    std::atomic_bool dtorCalled {false};
    std::atomic_bool odomTaskRunning {false};

    std::atomic_bool newMovement {false};
    std::atomic_bool doneLooping {true};
    std::atomic_bool doneLoopingSeen {true};

    /**
     * Wait for the distance setup (distancePid and anglePid) to settle.
     *
     * @param itimeout the timeout period for the distance movement
     *
     * @return the settle result
     */
    SettleResult waitForDistanceSettled(int itimeout);

    /**
     * Wait for the angle setup (anglePid) to settle.
     *
     * @param itimeout the timeout period for the angle movement
     *
     * @return the settle result
     */
    SettleResult waitForAngleSettled(int itimeout);

    /**
     * Stops all the controllers and the ChassisModel.
     */
    void stopAfterSettled();

    CrossplatformThread* task {nullptr};
    CrossplatformThread* odomTask {nullptr};

    static void trampoline(void *context);
    static void odomTrampoline(void *context);
    void loop();
    void odomLoop();
};
