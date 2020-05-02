#include <atomic>
#include <utility>

#include "libraidzero/controller/mecanumOdomController.hpp"
#include "libraidzero/controller/iodomController.hpp"
#include "libraidzero/geometry/pose2d.hpp"
#include "libraidzero/geometry/translation2d.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include "okapi/api/odometry/stateMode.hpp"
#include "okapi/api/util/timeUtil.hpp"

MecanumOdomController::MecanumOdomController(
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
        std::string msg("MecanumOdomController: The gear ratio cannot be zero! Check if you are using "
                        "integer division.");
        LOG_ERROR(msg);
        throw std::invalid_argument(msg);
    }
    model->setGearing(igearset.internalGearset);
    model->setEncoderUnits(AbstractMotor::encoderUnits::counts);
}

MecanumOdomController::~MecanumOdomController() {
    dtorCalled.store(true, std::memory_order_release);
    delete task;
}

void MecanumOdomController::loop() {
    LOG_INFO_S("Started MecanumOdomController task.");

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
                angleChange = (encVals[0] - encVals[1]) / 2.0; // left - right

                turnPid->step(angleChange);

                switch (turnType) {
                    case TurnType::PointTurn:
                        model->driveVectorVoltage(0, turnPid->getOutput());
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
                currentPose = Pose2d::fromOdomState(odometry->getState());

                std::string message = "MecanumController: Odom Pose=" + currentPose.toString();
                LOG_INFO(message);

                // Find the direction the drive should move towards in global coordinates
                auto directionVector = targetTranslation - currentPose.translation();
                // Use the same vector to find the distance to the target
                double distance = directionVector.norm().convert(meter);

                // Should always be negated since setpoint is always 0, and distance
                // is always positive. The direction vector only needs the magnitude.
                double distanceOutput = -distancePid->step(distance);

                directionVector /= distance;
                std::cout << directionVector.norm().convert(meter) << " should be 1" << std::endl;

                directionVector *= distanceOutput;
                directionVector = directionVector.rotateBy(-currentPose.rotation());

                double angleOutput = -anglePid->step(
                    currentPose.rotation().angle().convert(radian));

                std::cout << "Distance PID: " << distanceOutput << " | Angle PID: " << angleOutput << std::endl;

                model->xArcade(directionVector.x().convert(meter), 
                    directionVector.y().convert(meter), angleOutput);
                break;
            }
            case ControlMode::None:
                break;
            }
            lastMode = mode;
        }
        
        rate->delayUntil(10_ms);
    }
    LOG_INFO_S("Stopped MecanumOdomController task.");
}

void MecanumOdomController::driveForDistance(QLength idistance, int itimeout) {
    driveForDistanceAsync(idistance);
    waitUntilSettled(itimeout);
}

void MecanumOdomController::driveForDistanceAsync(QLength idistance) {
    distancePid->reset();
    anglePid->reset();
    distancePid->flipDisable(false);
    anglePid->flipDisable(false);
    turnPid->flipDisable(true);
    strafeDistancePid->flipDisable(true);
    strafeAnglePid->flipDisable(true);

    mode = ControlMode::Distance;

    const double newTarget = idistance.convert(meter) * scales.straight * gearsetRatioPair.ratio;

    LOG_INFO("MecanumOdomController: moving " + std::to_string(newTarget) + " motor ticks");

    distancePid->setTarget(newTarget);
    anglePid->setTarget(0);

    doneLooping.store(false, std::memory_order_release);
    newMovement.store(true, std::memory_order_release);
}

void MecanumOdomController::driveToPoint(const Point& ipoint, bool ibackwards, int itimeout) {
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

    LOG_INFO("MecanumOdomController: Computed length of " +
           std::to_string(length.convert(meter)) + " meters and angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("MecanumOdomController: Turning " + std::to_string(angle.convert(degree)) +
                 " degrees");
        turnAngle(angle, TurnType::PointTurn, itimeout);
    }

    if (length.abs() > distanceThreshold) {
        LOG_INFO("MecanumOdomController: Driving " +
             std::to_string((length).convert(meter)) + " meters");
        driveForDistance(length, itimeout);
    }
}

void MecanumOdomController::turnAngle(QAngle iangle, TurnType iturnType, int itimeout) {
    turnAngleAsync(iangle, iturnType);
    waitUntilSettled(itimeout);
}

void MecanumOdomController::turnAngleAsync(QAngle iangle, TurnType iturnType) {
    LOG_INFO("MecanumOdomController: turning " + std::to_string(iangle.convert(degree)) +
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

    LOG_INFO("MecanumOdomController: turning " + std::to_string(newAngle) + " motor ticks");

    turnPid->setTarget(newAngle);

    doneLooping.store(false, std::memory_order_release);
    newMovement.store(true, std::memory_order_release);
}

void MecanumOdomController::turnToAngle(QAngle iangle, TurnType iturnType, int itimeout) {
    waitForOdomTask();

    const auto angle = iangle - odometry->getState(StateMode::FRAME_TRANSFORMATION).theta;

    LOG_INFO("MecanumOdomController: Computed angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("MecanumOdomController: Turning " + std::to_string(angle.convert(degree)) +
                 " degrees");
        turnAngle(angle, iturnType, itimeout);
    }
}

void MecanumOdomController::turnToPoint(const Point& ipoint, int itimeout) {
    waitForOdomTask();

    const auto angle = OdomMath::computeAngleToPoint(ipoint.inFT(StateMode::FRAME_TRANSFORMATION),
                                                     odometry->getState(StateMode::FRAME_TRANSFORMATION));

    LOG_INFO("MecanumOdomController: Computed angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("MecanumOdomController: Turning " + std::to_string(angle.convert(degree)) +
                 " degrees");
        turnAngle(angle, TurnType::PointTurn, itimeout);
    }
}

void MecanumOdomController::strafeToPose(const Pose2d& ipose) {
    waitForOdomTask();

    LOG_INFO("MecanumOdomController: strafing to " + ipose.toString());

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

OdomState MecanumOdomController::getState() {
    return odometry->getState();
}

bool MecanumOdomController::isSettled() {
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

MecanumOdomController::SettleResult MecanumOdomController::waitUntilSettled(int itimeout) {
    LOG_INFO_S("MecanumOdomController: Waiting to settle");

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

    LOG_INFO_S("MecanumOdomController: Done waiting to settle");

    return result;
}

MecanumOdomController::SettleResult MecanumOdomController::waitForDistanceSettled(int itimeout) {
    LOG_INFO_S("MecanumOdomController: Waiting to settle in distance mode");

    uint32_t now = pros::millis();
    bool timeLeft = (pros::millis() - now < itimeout);
    bool settled = distancePid->isSettled() && anglePid->isSettled();
    auto rate = timeUtil.getRate();
    while (!settled && timeLeft) {
        if (mode != ControlMode::Distance) {
            // False will cause the loop to re-enter the switch
            LOG_WARN_S("MecanumOdomController: Mode changed to angle while waiting in distance!");
            return SettleResult::NotSettled;
        }
        settled = distancePid->isSettled() && anglePid->isSettled();
        timeLeft = (pros::millis() - now < itimeout);
        rate->delayUntil(10_ms);
    }
    return settled ? SettleResult::Settled : SettleResult::Timeout;
}

MecanumOdomController::SettleResult MecanumOdomController::waitForAngleSettled(int itimeout) {
    LOG_INFO_S("MecanumOdomController: Waiting to settle in angle mode");

    uint32_t now = pros::millis();
    bool timeLeft = (pros::millis() - now < itimeout);
    bool settled = turnPid->isSettled();
    auto rate = timeUtil.getRate();
    while (!settled && timeLeft) {
        if (mode != ControlMode::Angle) {
            // False will cause the loop to re-enter the switch
            LOG_WARN_S("MecanumOdomController: Mode changed to distance while waiting in angle!");
            return SettleResult::NotSettled;
        }
        settled = turnPid->isSettled();
        timeLeft = (pros::millis() - now < itimeout);
        rate->delayUntil(10_ms);
    }
    return settled ? SettleResult::Settled : SettleResult::Timeout;
}

MecanumOdomController::SettleResult MecanumOdomController::waitForStrafeSettled(int itimeout) {
    LOG_INFO_S("MecanumOdomController: Waiting to settle in strafe mode");

    uint32_t now = pros::millis();
    bool timeLeft = (pros::millis() - now < itimeout);
    bool settled = strafeDistancePid->isSettled() && strafeAnglePid->isSettled();
    auto rate = timeUtil.getRate();
    while (!settled && timeLeft) {
        if (mode != ControlMode::Distance) {
            // False will cause the loop to re-enter the switch
            LOG_WARN_S("MecanumOdomController: Mode changed to distance/angle while waiting in strafe!");
            return SettleResult::NotSettled;
        }
        settled = strafeDistancePid->isSettled() && strafeAnglePid->isSettled();
        timeLeft = (pros::millis() - now < itimeout);
        rate->delayUntil(10_ms);
    }
    return settled ? SettleResult::Settled : SettleResult::Timeout;
}

void MecanumOdomController::stopAfterSettled() {
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    turnPid->flipDisable(true);
    strafeDistancePid->flipDisable(true);
    strafeAnglePid->flipDisable(true);
    model->stop();
}

void MecanumOdomController::stop() {
    LOG_INFO_S("MecanumOdomController: Stopping");

    mode = ControlMode::None;
    doneLooping.store(true, std::memory_order_release);
    stopAfterSettled();
}

void MecanumOdomController::setMaxVoltage(int imaxVoltage) {
    model->setMaxVoltage(imaxVoltage);
}

int MecanumOdomController::getMaxVoltage() const {
    return static_cast<int>(model->getMaxVoltage());
}

std::shared_ptr<ChassisModel> MecanumOdomController::getModel() {
    return model;
}

ChassisScales MecanumOdomController::getChassisScales() const {
    return scales;
}

AbstractMotor::GearsetRatioPair MecanumOdomController::getGearsetRatioPair() const {
    return gearsetRatioPair;
}

void MecanumOdomController::startThread() {
    if (task == nullptr) {
        task = new CrossplatformThread(trampoline, this, "MecanumOdomController");
    }
}

void MecanumOdomController::trampoline(void* context) {
    if (context != nullptr) {
        static_cast<MecanumOdomController*>(context)->loop();
    }
}

void MecanumOdomController::startOdomThread() {
    if (odomTask == nullptr) {
        odomTask = new CrossplatformThread(trampoline, this, "MecanumOdomController");
    }
}

void MecanumOdomController::odomTrampoline(void* context) {
    if (context != nullptr) {
        static_cast<MecanumOdomController*>(context)->odomLoop();
    }
}

void MecanumOdomController::odomLoop() {
    odomTaskRunning = true;

    LOG_INFO_S("Started MecanumOdomController odometry task.");

    auto rate = timeUtil.getRate();
    while (!dtorCalled.load(std::memory_order_acquire) && (odomTask->notifyTake(0) == 0U)) {
        odometry->step();
        rate->delayUntil(10_ms);
    }
    odomTaskRunning = false;
    LOG_INFO_S("Stopped MecanumOdomController odometry task.");
}

void MecanumOdomController::waitForOdomTask() {
    if (odomTaskRunning) {
        // Early exit to save calling getRate
        return;
    }

    auto rate = timeUtil.getRate();
    while (!odomTaskRunning) {
        LOG_INFO_S("MecanumOdomController: Waiting for odometry task to start.");
        rate->delayUntil(10);
    }
}
