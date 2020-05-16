#include <atomic>
#include <utility>

#include "libraidzero/controller/odomController.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include "okapi/api/odometry/stateMode.hpp"
#include "okapi/api/util/timeUtil.hpp"

OdomController::OdomController(
    TimeUtil itimeUtil,
    std::shared_ptr<ChassisModel> imodel,
    std::shared_ptr<Odometry> iodometry,
    std::unique_ptr<IterativePosPIDController> idistancePid,
    std::unique_ptr<IterativePosPIDController> ianglePid,
    std::unique_ptr<IterativePosPIDController> iturnPid,
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
    gearsetRatioPair{igearset},
    scales{iscales},
    distanceThreshold{idistanceThreshold},
    turnThreshold{iturnThreshold},
    logger{std::move(ilogger)}
{
    if (igearset.ratio == 0) {
        std::string msg("OdomController: The gear ratio cannot be zero! Check if you are using "
                        "integer division.");
        LOG_ERROR(msg);
        throw std::invalid_argument(msg);
    }
    model->setGearing(igearset.internalGearset);
    model->setEncoderUnits(AbstractMotor::encoderUnits::counts);
}

OdomController::~OdomController() {
    dtorCalled.store(true, std::memory_order_release);
    delete task;
}

void OdomController::loop() {
    LOG_INFO_S("Started OdomController task.");

    auto encStartVals = model->getSensorVals();
    std::valarray<std::int32_t> encVals;
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
            default:
                break;
            }
            lastMode = mode;
        }
        
        rate->delayUntil(10_ms);
    }
    LOG_INFO_S("Stopped OdomController task.");
}

void OdomController::driveForDistance(QLength idistance, int itimeout) {
    driveForDistanceAsync(idistance);
    waitUntilSettled(itimeout);
}

void OdomController::driveForDistanceAsync(QLength idistance) {
    distancePid->reset();
    anglePid->reset();
    distancePid->flipDisable(false);
    anglePid->flipDisable(false);
    turnPid->flipDisable(true);

    mode = ControlMode::Distance;

    const double newTarget = idistance.convert(meter) * scales.straight * gearsetRatioPair.ratio;

    LOG_INFO("OdomController: moving " + std::to_string(newTarget) + " motor ticks");

    distancePid->setTarget(newTarget);
    anglePid->setTarget(0);

    doneLooping.store(false, std::memory_order_release);
    newMovement.store(true, std::memory_order_release);
}

void OdomController::driveToPoint(const Point& ipoint, bool ibackwards, int itimeout) {
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

    LOG_INFO("OdomController: Computed length of " +
           std::to_string(length.convert(meter)) + " meters and angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("OdomController: Turning " + std::to_string(angle.convert(degree)) +
                 " degrees");
        turnAngle(angle, TurnType::PointTurn, itimeout);
    }

    if (length.abs() > distanceThreshold) {
        LOG_INFO("OdomController: Driving " +
             std::to_string((length).convert(meter)) + " meters");
        driveForDistance(length, itimeout);
    }
}

void OdomController::turnAngle(QAngle iangle, TurnType iturnType, int itimeout) {
    turnAngleAsync(iangle, iturnType);
    waitUntilSettled(itimeout);
}

void OdomController::turnAngleAsync(QAngle iangle, TurnType iturnType) {
    LOG_INFO("OdomController: turning " + std::to_string(iangle.convert(degree)) +
            " degrees");

    turnPid->reset();
    turnPid->flipDisable(false);
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    mode = ControlMode::Angle;
    turnType = iturnType;

    const double newAngle =
        iangle.convert(degree) * scales.turn * gearsetRatioPair.ratio;

    LOG_INFO("OdomController: turning " + std::to_string(newAngle) + " motor ticks");

    turnPid->setTarget(newAngle);

    doneLooping.store(false, std::memory_order_release);
    newMovement.store(true, std::memory_order_release);
}

void OdomController::turnToAngle(QAngle iangle, TurnType iturnType, int itimeout) {
    waitForOdomTask();

    const auto angle = iangle - odometry->getState(StateMode::FRAME_TRANSFORMATION).theta;

    LOG_INFO("OdomController: Computed angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("OdomController: Turning " + std::to_string(angle.convert(degree)) +
                 " degrees");
        turnAngle(angle, iturnType, itimeout);
    }
}

void OdomController::turnToPoint(const Point& ipoint, int itimeout) {
    waitForOdomTask();

    const auto angle = OdomMath::computeAngleToPoint(ipoint.inFT(StateMode::FRAME_TRANSFORMATION),
                                                     odometry->getState(StateMode::FRAME_TRANSFORMATION));

    LOG_INFO("OdomController: Computed angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("OdomController: Turning " + std::to_string(angle.convert(degree)) +
                 " degrees");
        turnAngle(angle, TurnType::PointTurn, itimeout);
    }
}

void OdomController::setState(OdomState istate) {
    odometry->setState(istate);
}

OdomState OdomController::getState() {
    return odometry->getState();
}

bool OdomController::isSettled() {
    switch (mode) {
    case ControlMode::Distance:
        return distancePid->isSettled() && anglePid->isSettled();
    case ControlMode::Angle:
        return turnPid->isSettled();
    default:
        return true;
    }
}

OdomController::SettleResult OdomController::waitUntilSettled(int itimeout) {
    LOG_INFO_S("OdomController: Waiting to settle");

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

    LOG_INFO_S("OdomController: Done waiting to settle");

    return result;
}

OdomController::SettleResult OdomController::waitForDistanceSettled(int itimeout) {
    LOG_INFO_S("OdomController: Waiting to settle in distance mode");

    uint32_t now = pros::millis();
    bool timeLeft = (pros::millis() - now < itimeout);
    bool settled = distancePid->isSettled() && anglePid->isSettled();
    auto rate = timeUtil.getRate();
    while (!settled && timeLeft) {
        if (mode == ControlMode::Angle) {
            // False will cause the loop to re-enter the switch
            LOG_WARN_S("OdomController: Mode changed to angle while waiting in distance!");
            return SettleResult::NotSettled;
        }
        settled = distancePid->isSettled() && anglePid->isSettled();
        timeLeft = (pros::millis() - now < itimeout);
        rate->delayUntil(10_ms);
    }
    return settled ? SettleResult::Settled : SettleResult::Timeout;
}

OdomController::SettleResult OdomController::waitForAngleSettled(int itimeout) {
    LOG_INFO_S("OdomController: Waiting to settle in angle mode");

    uint32_t now = pros::millis();
    bool timeLeft = (pros::millis() - now < itimeout);
    bool settled = turnPid->isSettled();
    auto rate = timeUtil.getRate();
    while (!settled && timeLeft) {
        if (mode == ControlMode::Distance) {
            // False will cause the loop to re-enter the switch
            LOG_WARN_S("OdomController: Mode changed to distance while waiting in angle!");
            return SettleResult::NotSettled;
        }
        settled = turnPid->isSettled();
        timeLeft = (pros::millis() - now < itimeout);
        rate->delayUntil(10_ms);
    }
    return settled ? SettleResult::Settled : SettleResult::Timeout;
}

void OdomController::stopAfterSettled() {
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    turnPid->flipDisable(true);
    model->stop();
}

void OdomController::stop() {
    LOG_INFO_S("OdomController: Stopping");

    mode = ControlMode::None;
    doneLooping.store(true, std::memory_order_release);
    stopAfterSettled();
}

void OdomController::setMaxVoltage(int imaxVoltage) {
    model->setMaxVoltage(imaxVoltage);
}

int OdomController::getMaxVoltage() const {
    return static_cast<int>(model->getMaxVoltage());
}

std::shared_ptr<ChassisModel> OdomController::getModel() {
    return model;
}

ChassisScales OdomController::getChassisScales() const {
    return scales;
}

AbstractMotor::GearsetRatioPair OdomController::getGearsetRatioPair() const {
    return gearsetRatioPair;
}

CrossplatformThread* OdomController::getThread() const {
    return task;
}

CrossplatformThread* OdomController::getOdomThread() const {
    return odomTask;
}

void OdomController::startThread() {
    if (task == nullptr) {
        task = new CrossplatformThread(trampoline, this, "OdomController");
    }
}

void OdomController::trampoline(void* context) {
    if (context != nullptr) {
        static_cast<OdomController*>(context)->loop();
    }
}

void OdomController::startOdomThread() {
    if (odomTask == nullptr) {
        odomTask = new CrossplatformThread(odomTrampoline, this, "OdomController");
    }
}

void OdomController::odomTrampoline(void* context) {
    if (context != nullptr) {
        static_cast<OdomController*>(context)->odomLoop();
    }
}

void OdomController::odomLoop() {
    odomTaskRunning = true;

    LOG_INFO_S("Started OdomController odometry task.");

    auto rate = timeUtil.getRate();
    while (!dtorCalled.load(std::memory_order_acquire) && (odomTask->notifyTake(0) == 0U)) {
        odometry->step();
        rate->delayUntil(10_ms);
    }
    odomTaskRunning = false;
    LOG_INFO_S("Stopped OdomController odometry task.");
}

void OdomController::waitForOdomTask() {
    if (odomTaskRunning) {
        // Early exit to save calling getRate
        return;
    }

    auto rate = timeUtil.getRate();
    while (!odomTaskRunning) {
        LOG_INFO_S("OdomController: Waiting for odometry task to start.");
        rate->delayUntil(10);
    }
}
