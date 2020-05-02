#pragma once

#include "main.h"
#include "libraidzero/geometry/pose2d.hpp"
#include "libraidzero/controller/advancedOdomChassisController.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/odometry/odometry.hpp"
#include "pidController.hpp"

class MecanumController {
public:
    MecanumController(
        TimeUtil itimeUtil,
        std::shared_ptr<AdvancedOdomChassisController> odomChassisController,
        PIDController::Gains distanceGains, PIDController::Gains angleGains
    );
    ~MecanumController();

    void strafeToPose(Pose2d ipose);

    /**
     * Returns whether the controller has settled at the target. Determining what settling means is
     * implementation-dependent.
     *
     * If the controller is disabled, this method must return true.
     *
     * @return whether the controller is settled
     */
    bool isSettled();

    /**
     * Delays until the currently executing movement completes.
     */
    bool waitUntilSettled(int itimeout);
  
    /**
     * Resets the controller so it can start from 0 again properly. Keeps configuration from
     * before. This implementation also stops movement.
     */
    void reset();

    /**
     * Sets whether the controller is off or on. Turning the controller on after it was off will
     * NOT cause the controller to move to its last set target, unless it was reset in that time.
     *
     * @param iisDisabled whether the controller is disabled
     */
    void flipDisable(bool iisDisabled);
  
    /**
     * Returns whether the controller is currently disabled.
     *
     * @return whether the controller is currently disabled
     */
    bool isDisabled() const;

    /**
     * Starts the internal thread. This should not be called by normal users. 
     * This method is called by the `MecanumController` when making a new 
     * instance of this class.
     */
    void startThread();
  
    /**
     * @return The underlying thread handle.
     */
    CrossplatformThread *getThread() const;
protected:
    std::shared_ptr<AdvancedOdomChassisController> chassisController;

    std::shared_ptr<Logger> logger;
    std::shared_ptr<XDriveModel> model;
    std::shared_ptr<Odometry> odometry;
    ChassisScales scales;
    AbstractMotor::GearsetRatioPair pair;
    TimeUtil timeUtil;

    int timeout = 0;

    std::unique_ptr<PIDController> distancePid;
    std::unique_ptr<PIDController> anglePid;

    Pose2d targetPose;

    std::atomic_bool isRunning{false};
    std::atomic_bool disabled{false};
    std::atomic_bool dtorCalled{false};
    CrossplatformThread *task{nullptr};

    static void trampoline(void *context);
    void loop();

    void executeStrafe(const Pose2d& target, std::unique_ptr<AbstractRate> rate);

    enum class ModeType{ None, Strafing };
    ModeType mode{ModeType::None};
};