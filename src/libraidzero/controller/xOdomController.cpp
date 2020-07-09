#include <atomic>
#include <utility>

#include "libraidzero/controller/xOdomController.hpp"
#include "libraidzero/controller/iodomController.hpp"
#include "libraidzero/geometry/pose2d.hpp"
#include "libraidzero/geometry/rotation2d.hpp"
#include "libraidzero/geometry/translation2d.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include "okapi/api/odometry/stateMode.hpp"
#include "okapi/api/util/timeUtil.hpp"

XOdomController::XOdomController(
    TimeUtil itimeUtil,
    std::shared_ptr<XDriveModel> imodel,
    std::shared_ptr<Odometry> iodometry,
    std::unique_ptr<IterativePosPIDController> idistancePid,
    std::unique_ptr<IterativePosPIDController> ianglePid,
    std::unique_ptr<IterativePosPIDController> iturnPid,
    std::unique_ptr<IterativePosPIDController> istrafeDistancePid,
    std::unique_ptr<IterativePosPIDController> istrafeAnglePid,
    const AbstractMotor::GearsetRatioPair& igearset,
    const ChassisScales& iscales,
    QLength idistanceThreshold,
    QAngle iturnThreshold,
    std::shared_ptr<Logger> ilogger
) : timeUtil{std::move(itimeUtil)},
    model{std::move(imodel)},
    odometry{std::move(iodometry)},
    distancePid{std::move(idistancePid)},
    anglePid{std::move(ianglePid)},
    turnPid{std::move(iturnPid)},
    strafeDistancePid{std::move(istrafeDistancePid)},
    strafeAnglePid{std::move(istrafeAnglePid)},
    gearsetRatioPair{igearset},
    scales{iscales},
    distanceThreshold{idistanceThreshold},
    turnThreshold{iturnThreshold},
    logger{std::move(ilogger)}
{
    if (igearset.ratio == 0) {
        std::string msg("XOdomController: The gear ratio cannot be zero! Check if you are using "
                        "integer division.");
        LOG_ERROR(msg);
        throw std::invalid_argument(msg);
    }
    model->setGearing(igearset.internalGearset);
    model->setEncoderUnits(AbstractMotor::encoderUnits::counts);
}

XOdomController::~XOdomController() {
    dtorCalled.store(true, std::memory_order_release);
    delete task;
}

void XOdomController::loop() {
    LOG_INFO_S("Started XOdomController task.");

    auto encStartVals = model->getSensorVals();
    std::valarray<std::int32_t> encVals;
    Pose2d currentPose;
    Translation2d targetTranslation;
    double distanceElapsed = 0.0;
    double angleChange = 0.0;
    auto rate = timeUtil.getRate();
    ControlMode lastMode = ControlMode::None;

    while (!dtorCalled.load(std::memory_order_acquire) && (task->notifyTake(0) == 0U)) {
        /**
         * doneLooping is set to false by distance and turning methods and then 
         * set to true by waitUntilSettled
         */
        if (doneLooping.load(std::memory_order_acquire)) {
            doneLoopingSeen.store(true, std::memory_order_release);
        } else {
            if (lastMode != mode || newMovement.load(std::memory_order_acquire)) {
                encStartVals = model->getSensorVals();
                if (mode == ControlMode::P2PStrafe) {
                    targetTranslation = targetPose.translation();
                }
                newMovement.store(false, std::memory_order_release);
            }
            switch (mode) {
            case ControlMode::Distance:
                encVals = model->getSensorVals() - encStartVals;
                distanceElapsed = static_cast<double>((encVals[0] + encVals[1])) / 2.0;
                angleChange = static_cast<double>(encVals[0] - encVals[1]); // left - right

                distancePid->step(distanceElapsed);
                anglePid->step(angleChange);
                model->driveVectorVoltage(distancePid->getOutput(), anglePid->getOutput());
                break;
            case ControlMode::Angle:
                encVals = model->getSensorVals() - encStartVals;
                angleChange = (encVals[1] - encVals[0]) / 2.0; // right - left

                turnPid->step(angleChange);

                switch (turnType) {
                    case TurnType::PointTurn:
                        model->driveVectorVoltage(0, -turnPid->getOutput());
                        break;
                    case TurnType::LeftPivot:
                        model->tank(0, -turnPid->getOutput());
                        break;
                    case TurnType::RightPivot:
                        model->tank(turnPid->getOutput(), 0);
                        break;
                }
                break;
            case ControlMode::P2PStrafe: {
                // Use cartesian to flip x & y axes since odom works in a
                // different frame
                currentPose = Pose2d::fromOdomState(odometry->getState(StateMode::CARTESIAN));

                std::string message = "XOdomController: Odom Pose=" + currentPose.toString();
                LOG_INFO(message);

                // Find the direction the drive should move towards in global coordinates
                auto directionVector = targetTranslation - currentPose.translation();

                // Use the same vector to find the distance to the target
                double distance = directionVector.norm().convert(meter);

                // Should always be negated since setpoint is always 0, and distance
                // is always positive. The direction vector only needs the magnitude.
                double distanceOutput = -strafeDistancePid->step(distance);

                // Normalize the vector & scale it by the 
                directionVector /= distance;
                directionVector *= distanceOutput;
                directionVector = directionVector.rotateBy(-currentPose.rotation());
                //std::cout << "DirX: " << directionVector.x().convert(meter) << ", DirY:" << directionVector.y().convert(meter) << std::endl;

                double angleOutput = -strafeAnglePid->step(
                    currentPose.rotation().angle().convert(radian));

                //std::cout << "Distance PID: " << distanceOutput << " | Angle PID: " << angleOutput << std::endl;

                model->xArcade(
                    directionVector.x().convert(meter),
                    directionVector.y().convert(meter), 
                    angleOutput
                );
                break;
            }
            case ControlMode::None:
                break;
            }
            lastMode = mode;
        }
        
        rate->delayUntil(10_ms);
    }
    LOG_INFO_S("Stopped XOdomController task.");
}

void XOdomController::driveForDistance(QLength idistance, int itimeout) {
    driveForDistanceAsync(idistance);
    waitUntilSettled(itimeout);
}

void XOdomController::driveForDistanceAsync(QLength idistance) {
    distancePid->reset();
    anglePid->reset();
    distancePid->flipDisable(false);
    anglePid->flipDisable(false);
    turnPid->flipDisable(true);
    strafeDistancePid->flipDisable(true);
    strafeAnglePid->flipDisable(true);

    mode = ControlMode::Distance;

    const double newTarget = idistance.convert(meter) * scales.straight * gearsetRatioPair.ratio;

    LOG_INFO("XOdomController: moving " + std::to_string(newTarget) + " motor ticks");

    distancePid->setTarget(newTarget);
    anglePid->setTarget(0);

    doneLooping.store(false, std::memory_order_release);
    newMovement.store(true, std::memory_order_release);
}

void XOdomController::driveToPoint(const Point& ipoint, bool ibackwards, int itimeout) {
    waitForOdomTask();

    auto pair = OdomMath::computeDistanceAndAngleToPoint(
        ipoint.inFT(StateMode::FRAME_TRANSFORMATION), 
        odometry->getState(StateMode::FRAME_TRANSFORMATION)
    );
    auto length = pair.first;
    auto angle = pair.second;

    if (ibackwards) {
        length *= -1;
        angle += 180_deg;
    }

    LOG_INFO("XOdomController: Computed length of " +
           std::to_string(length.convert(meter)) + " meters and angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("XOdomController: Turning " + std::to_string(angle.convert(degree)) +
                 " degrees");
        turnAngle(angle, TurnType::PointTurn, itimeout);
    }

    if (length.abs() > distanceThreshold) {
        LOG_INFO("XOdomController: Driving " +
             std::to_string((length).convert(meter)) + " meters");
        driveForDistance(length, itimeout);
    }
}

void XOdomController::turnAngle(QAngle iangle, TurnType iturnType, int itimeout) {
    turnAngleAsync(iangle, iturnType);
    waitUntilSettled(itimeout);
}

void XOdomController::turnAngleAsync(QAngle iangle, TurnType iturnType) {
    LOG_INFO("XOdomController: turning " + std::to_string(iangle.convert(degree)) +
            " degrees");

    turnPid->reset();
    turnPid->flipDisable(false);
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    strafeDistancePid->flipDisable(true);
    strafeAnglePid->flipDisable(true);

    mode = ControlMode::Angle;
    turnType = iturnType;

    const double newAngle =
        iangle.convert(degree) * scales.turn * gearsetRatioPair.ratio;

    LOG_INFO("XOdomController: turning " + std::to_string(newAngle) + " motor ticks");

    turnPid->setTarget(newAngle);

    doneLooping.store(false, std::memory_order_release);
    newMovement.store(true, std::memory_order_release);
}

void XOdomController::turnToAngle(QAngle iangle, TurnType iturnType, int itimeout) {
    waitForOdomTask();

    const auto angle = iangle - odometry->getState(StateMode::FRAME_TRANSFORMATION).theta;

    LOG_INFO("XOdomController: Computed angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("XOdomController: Turning " + std::to_string(angle.convert(degree)) +
                 " degrees");
        turnAngle(angle, iturnType, itimeout);
    }
}

void XOdomController::turnToPoint(const Point& ipoint, int itimeout) {
    waitForOdomTask();

    const auto angle = -OdomMath::computeAngleToPoint(ipoint.inFT(StateMode::CARTESIAN),
                                                     odometry->getState(StateMode::FRAME_TRANSFORMATION));

    LOG_INFO("XOdomController: Computed angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("XOdomController: Turning " + std::to_string(angle.convert(degree)) +
                 " degrees");
        turnAngle(angle, TurnType::PointTurn, itimeout);
    }
}

void XOdomController::strafeToPoint(const Point& ipoint) {
    strafeToPose(
        Pose2d{
            Translation2d{ipoint.x, ipoint.y}, 
            Rotation2d{-1 * odometry->getState().theta}
        }
    );
}

void XOdomController::strafeToPose(const Pose2d& ipose) {
    waitForOdomTask();

    LOG_INFO("XOdomController: strafing to " + ipose.toString());

    strafeDistancePid->reset();
    strafeDistancePid->flipDisable(false);
    strafeDistancePid->setTarget(0.0);
    strafeAnglePid->reset();
    strafeAnglePid->flipDisable(false);
    strafeAnglePid->setTarget(ipose.rotation().angle().convert(radian));
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);

    mode = ControlMode::P2PStrafe;
    targetPose = ipose;

    doneLooping.store(false, std::memory_order_release);
    newMovement.store(true, std::memory_order_release);
}

void XOdomController::setState(OdomState istate) {
    odometry->setState(istate);
}

OdomState XOdomController::getState() {
    auto s = odometry->getState(StateMode::CARTESIAN);
    s.y = -s.y;
    return s;
}

bool XOdomController::isSettled() {
    switch (mode) {
    case ControlMode::Distance:
        return distancePid->isSettled() && anglePid->isSettled();
    case ControlMode::Angle:
        return turnPid->isSettled();
    case ControlMode::P2PStrafe:
        return strafeDistancePid->isSettled() && strafeAnglePid->isSettled();
    default:
        return true;
    }
}

XOdomController::SettleResult XOdomController::waitUntilSettled(int itimeout) {
    LOG_INFO_S("XOdomController: Waiting to settle");

    // make it as large as possible, so effectively no timeout
    if (itimeout == 0) {
        itimeout = ~itimeout;
    }

    SettleResult result = SettleResult::NotSettled;
    while (result == SettleResult::NotSettled) {
        switch (mode) {
        case ControlMode::Distance:
            result = waitForDistanceSettled(itimeout);
            break;
        case ControlMode::Angle:
            result = waitForAngleSettled(itimeout);
            break;
        case ControlMode::P2PStrafe:
            result = waitForStrafeSettled(itimeout);
            break;
        default:
            result = SettleResult::Settled;
            break;
        }
    }

    // Order here is important
    mode = ControlMode::None;
    doneLooping.store(true, std::memory_order_release);
    doneLoopingSeen.store(false, std::memory_order_release);

    auto rate = timeUtil.getRate();
    // Wait for the thread to finish if it happens to be writing to motors
    while (!doneLoopingSeen.load(std::memory_order_acquire)) {
        rate->delayUntil(10_ms);
    }

    // Stop after the thread has run at least once
    stopAfterSettled();

    LOG_INFO_S("XOdomController: Done waiting to settle");

    return result;
}

XOdomController::SettleResult XOdomController::waitForDistanceSettled(int itimeout) {
    LOG_INFO_S("XOdomController: Waiting to settle in distance mode");

    uint32_t now = pros::millis();
    bool timeLeft = (pros::millis() - now < itimeout);
    bool settled = distancePid->isSettled() && anglePid->isSettled();
    auto rate = timeUtil.getRate();
    while (!settled && timeLeft) {
        if (mode != ControlMode::Distance) {
            // False will cause the loop to re-enter the switch
            LOG_WARN_S("XOdomController: Mode changed to angle while waiting in distance!");
            return SettleResult::NotSettled;
        }
        settled = distancePid->isSettled() && anglePid->isSettled();
        timeLeft = (pros::millis() - now < itimeout);
        rate->delayUntil(10_ms);
    }
    return settled ? SettleResult::Settled : SettleResult::Timeout;
}

XOdomController::SettleResult XOdomController::waitForAngleSettled(int itimeout) {
    LOG_INFO_S("XOdomController: Waiting to settle in angle mode");

    uint32_t now = pros::millis();
    bool timeLeft = (pros::millis() - now < itimeout);
    bool settled = turnPid->isSettled();
    auto rate = timeUtil.getRate();
    while (!settled && timeLeft) {
        if (mode != ControlMode::Angle) {
            // False will cause the loop to re-enter the switch
            LOG_WARN_S("XOdomController: Mode changed to distance while waiting in angle!");
            return SettleResult::NotSettled;
        }
        settled = turnPid->isSettled();
        timeLeft = (pros::millis() - now < itimeout);
        rate->delayUntil(10_ms);
    }
    return settled ? SettleResult::Settled : SettleResult::Timeout;
}

XOdomController::SettleResult XOdomController::waitForStrafeSettled(int itimeout) {
    LOG_INFO_S("XOdomController: Waiting to settle in strafe mode");

    uint32_t now = pros::millis();
    bool timeLeft = (pros::millis() - now < itimeout);
    bool settled = strafeDistancePid->isSettled() && strafeAnglePid->isSettled();
    auto rate = timeUtil.getRate();
    while (!settled && timeLeft) {
        if (mode != ControlMode::P2PStrafe) {
            // False will cause the loop to re-enter the switch
            LOG_WARN_S("XOdomController: Mode changed to distance/angle while waiting in strafe!");
            return SettleResult::NotSettled;
        }
        settled = strafeDistancePid->isSettled() && strafeAnglePid->isSettled();
        timeLeft = (pros::millis() - now < itimeout);
        rate->delayUntil(10_ms);
    }
    return settled ? SettleResult::Settled : SettleResult::Timeout;
}

void XOdomController::stopAfterSettled() {
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    turnPid->flipDisable(true);
    strafeDistancePid->flipDisable(true);
    strafeAnglePid->flipDisable(true);
    model->stop();
}

void XOdomController::stop() {
    LOG_INFO_S("XOdomController: Stopping");

    mode = ControlMode::None;
    doneLooping.store(true, std::memory_order_release);
    stopAfterSettled();
}

void XOdomController::setMaxVoltage(int imaxVoltage) {
    model->setMaxVoltage(imaxVoltage);
}

int XOdomController::getMaxVoltage() const {
    return static_cast<int>(model->getMaxVoltage());
}

std::shared_ptr<ChassisModel> XOdomController::getModel() {
    return model;
}

ChassisScales XOdomController::getChassisScales() const {
    return scales;
}

AbstractMotor::GearsetRatioPair XOdomController::getGearsetRatioPair() const {
    return gearsetRatioPair;
}

CrossplatformThread* XOdomController::getThread() const {
    return task;
}

CrossplatformThread* XOdomController::getOdomThread() const {
    return odomTask;
}

void XOdomController::startThread() {
    if (task == nullptr) {
        task = new CrossplatformThread(trampoline, this, "XOdomController");
    }
}

void XOdomController::trampoline(void* context) {
    if (context != nullptr) {
        static_cast<XOdomController*>(context)->loop();
    }
}

void XOdomController::startOdomThread() {
    if (odomTask == nullptr) {
        odomTask = new CrossplatformThread(odomTrampoline, this, "XOdomController");
    }
}

void XOdomController::odomTrampoline(void* context) {
    if (context != nullptr) {
        static_cast<XOdomController*>(context)->odomLoop();
    }
}

void XOdomController::odomLoop() {
    odomTaskRunning = true;

    LOG_INFO_S("Started XOdomController odometry task.");

    auto rate = timeUtil.getRate();
    while (!dtorCalled.load(std::memory_order_acquire) && (odomTask->notifyTake(0) == 0U)) {
        odometry->step();
        rate->delayUntil(10_ms);
    }
    odomTaskRunning = false;
    LOG_INFO_S("Stopped XOdomController odometry task.");
}

void XOdomController::waitForOdomTask() {
    if (odomTaskRunning) {
        // Early exit to save calling getRate
        return;
    }

    auto rate = timeUtil.getRate();
    while (!odomTaskRunning) {
        LOG_INFO_S("XOdomController: Waiting for odometry task to start.");
        rate->delayUntil(10);
    }
}
