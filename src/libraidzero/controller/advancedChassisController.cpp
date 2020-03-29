/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "libraidzero/controller/advancedChassisController.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <cmath>
#include <utility>

using namespace okapi;

AdvancedChassisController::AdvancedChassisController(
    TimeUtil itimeUtil,
    std::shared_ptr<ChassisModel> ichassisModel,
    std::unique_ptr<IterativePosPIDController> idistanceController,
    std::unique_ptr<IterativePosPIDController> iturnController,
    std::unique_ptr<IterativePosPIDController> iangleController,
    const AbstractMotor::GearsetRatioPair &igearset,
    const ChassisScales &iscales,
    std::shared_ptr<Logger> ilogger)
    : logger(std::move(ilogger)),
      chassisModel(std::move(ichassisModel)),
      timeUtil(std::move(itimeUtil)),
      distancePid(std::move(idistanceController)),
      turnPid(std::move(iturnController)),
      anglePid(std::move(iangleController)),
      scales(iscales),
      gearsetRatioPair(igearset) {
    if (igearset.ratio == 0) {
        std::string msg("AdvancedChassisController: The gear ratio cannot be zero! Check if you are using "
                                        "integer division.");
        LOG_ERROR(msg);
        throw std::invalid_argument(msg);
    }

    chassisModel->setGearing(igearset.internalGearset);
    chassisModel->setEncoderUnits(AbstractMotor::encoderUnits::counts);
}

AdvancedChassisController::~AdvancedChassisController() {
    dtorCalled.store(true, std::memory_order_release);
    delete task;
}

void AdvancedChassisController::loop() {
    LOG_INFO_S("Started AdvancedChassisController task.");

    auto encStartVals = chassisModel->getSensorVals();
    std::valarray<std::int32_t> encVals;
    double distanceElapsed = 0, angleChange = 0;
    modeType pastMode = none;
    auto rate = timeUtil.getRate();

    while (!dtorCalled.load(std::memory_order_acquire) && !task->notifyTake(0)) {
        /**
         * doneLooping is set to false by moveDistanceAsync and turnAngleAsync and then set to true by
         * waitUntilSettled
         */
        if (doneLooping.load(std::memory_order_acquire)) {
            doneLoopingSeen.store(true, std::memory_order_release);
        } else {
            if (mode != pastMode || newMovement.load(std::memory_order_acquire)) {
                encStartVals = chassisModel->getSensorVals();
                newMovement.store(false, std::memory_order_release);
            }

            switch (mode) {
            case distance:
                encVals = chassisModel->getSensorVals() - encStartVals;
                distanceElapsed = static_cast<double>((encVals[0] + encVals[1])) / 2.0;
                angleChange = static_cast<double>(encVals[0] - encVals[1]);

                distancePid->step(distanceElapsed);
                anglePid->step(angleChange);

                if (velocityMode) {
                    chassisModel->driveVector(distancePid->getOutput(), anglePid->getOutput());
                } else {
                    chassisModel->driveVectorVoltage(distancePid->getOutput(), anglePid->getOutput());
                }

                break;

            case angle:
                encVals = chassisModel->getSensorVals() - encStartVals;
                angleChange = (encVals[0] - encVals[1]) / 2.0;

                turnPid->step(angleChange);

                if (velocityMode) {
                    chassisModel->driveVector(0, turnPid->getOutput());
                } else {
                    chassisModel->driveVectorVoltage(0, turnPid->getOutput());
                }

                break;

            default:
                break;
            }

            pastMode = mode;
        }

        rate->delayUntil(threadSleepTime);
    }

    stop();

    LOG_INFO_S("Stopped AdvancedChassisController task.");
}

void AdvancedChassisController::trampoline(void *context) {
    if (context) {
        static_cast<AdvancedChassisController *>(context)->loop();
    }
}

void AdvancedChassisController::moveDistanceAsync(const QLength itarget) {
    LOG_INFO("AdvancedChassisController: moving " + std::to_string(itarget.convert(meter)) + " meters");

    distancePid->reset();
    anglePid->reset();
    distancePid->flipDisable(false);
    anglePid->flipDisable(false);
    turnPid->flipDisable(true);
    mode = distance;

    const double newTarget = itarget.convert(meter) * scales.straight * gearsetRatioPair.ratio;

    LOG_INFO("AdvancedChassisController: moving " + std::to_string(newTarget) + " motor ticks");

    distancePid->setTarget(newTarget);
    anglePid->setTarget(0);

    doneLooping.store(false, std::memory_order_release);
    newMovement.store(true, std::memory_order_release);
}

void AdvancedChassisController::moveRawAsync(const double itarget) {
    // Divide by straightScale so the final result turns back into motor ticks
    moveDistanceAsync((itarget / scales.straight) * meter);
}

void AdvancedChassisController::moveDistance(const QLength itarget) {
    moveDistanceAsync(itarget);
    waitUntilSettled();
}

void AdvancedChassisController::moveRaw(const double itarget) {
    // Divide by straightScale so the final result turns back into motor ticks
    moveDistance((itarget / scales.straight) * meter);
}

void AdvancedChassisController::turnAngleAsync(const QAngle idegTarget) {
    LOG_INFO("AdvancedChassisController: turning " + std::to_string(idegTarget.convert(degree)) +
           " degrees");

    turnPid->reset();
    turnPid->flipDisable(false);
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    mode = angle;

    const double newTarget =
        idegTarget.convert(degree) * scales.turn * gearsetRatioPair.ratio * boolToSign(normalTurns);

    LOG_INFO("AdvancedChassisController: turning " + std::to_string(newTarget) + " motor ticks");

    turnPid->setTarget(newTarget);

    doneLooping.store(false, std::memory_order_release);
    newMovement.store(true, std::memory_order_release);
}

void AdvancedChassisController::turnRawAsync(const double idegTarget) {
    // Divide by turnScale so the final result turns back into motor ticks
    turnAngleAsync((idegTarget / scales.turn) * degree);
}

void AdvancedChassisController::turnAngle(const QAngle idegTarget) {
    turnAngleAsync(idegTarget);
    waitUntilSettled();
}

void AdvancedChassisController::turnRaw(const double idegTarget) {
    // Divide by turnScale so the final result turns back into motor ticks
    turnAngle((idegTarget / scales.turn) * degree);
}

void AdvancedChassisController::setTurnsMirrored(const bool ishouldMirror) {
    normalTurns = !ishouldMirror;
}

bool AdvancedChassisController::isSettled() {
    switch (mode) {
    case distance:
        return distancePid->isSettled() && anglePid->isSettled();

    case angle:
        return turnPid->isSettled();

    default:
        return true;
    }
}

void AdvancedChassisController::waitUntilSettled() {
    LOG_INFO_S("AdvancedChassisController: Waiting to settle");

    bool completelySettled = false;

    while (!completelySettled) {
        switch (mode) {
        case distance:
            completelySettled = waitForDistanceSettled();
            break;

        case angle:
            completelySettled = waitForAngleSettled();
            break;

        default:
            completelySettled = true;
            break;
        }
    }

    // Order here is important
    mode = none;
    doneLooping.store(true, std::memory_order_release);
    doneLoopingSeen.store(false, std::memory_order_release);

    // Wait for the thread to finish if it happens to be writing to motors
    auto rate = timeUtil.getRate();
    while (!doneLoopingSeen.load(std::memory_order_acquire)) {
        rate->delayUntil(threadSleepTime);
    }

    // Stop after the thread has run at least once
    stopAfterSettled();

    LOG_INFO_S("AdvancedChassisController: Done waiting to settle");
}

bool AdvancedChassisController::waitForDistanceSettled() {
    LOG_INFO_S("AdvancedChassisController: Waiting to settle in distance mode");

    auto rate = timeUtil.getRate();
    while (!(distancePid->isSettled() && anglePid->isSettled())) {
        if (mode == angle) {
            // False will cause the loop to re-enter the switch
            LOG_WARN_S("AdvancedChassisController: Mode changed to angle while waiting in distance!");
            return false;
        }

        rate->delayUntil(10_ms);
    }

    // True will cause the loop to exit
    return true;
}

bool AdvancedChassisController::waitForAngleSettled() {
    LOG_INFO_S("AdvancedChassisController: Waiting to settle in angle mode");

    auto rate = timeUtil.getRate();
    while (!turnPid->isSettled()) {
        if (mode == distance) {
            // False will cause the loop to re-enter the switch
            LOG_WARN_S("AdvancedChassisController: Mode changed to distance while waiting in angle!");
            return false;
        }

        rate->delayUntil(10_ms);
    }

    // True will cause the loop to exit
    return true;
}

void AdvancedChassisController::stopAfterSettled() {
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    turnPid->flipDisable(true);
    chassisModel->stop();
}

ChassisScales AdvancedChassisController::getChassisScales() const {
    return scales;
}

AbstractMotor::GearsetRatioPair AdvancedChassisController::getGearsetRatioPair() const {
    return gearsetRatioPair;
}

void AdvancedChassisController::setVelocityMode(bool ivelocityMode) {
    velocityMode = ivelocityMode;
}

void AdvancedChassisController::setGains(const okapi::IterativePosPIDController::Gains &idistanceGains,
                                         const okapi::IterativePosPIDController::Gains &iturnGains,
                                         const okapi::IterativePosPIDController::Gains &iangleGains) {
    distancePid->setGains(idistanceGains);
    turnPid->setGains(iturnGains);
    anglePid->setGains(iangleGains);
}

std::tuple<IterativePosPIDController::Gains,
           IterativePosPIDController::Gains,
           IterativePosPIDController::Gains>
AdvancedChassisController::getGains() const {
    return std::make_tuple(distancePid->getGains(), turnPid->getGains(), anglePid->getGains());
}

void AdvancedChassisController::startThread() {
    if (!task) {
        task = new CrossplatformThread(trampoline, this, "AdvancedChassisController");
    }
}

CrossplatformThread *AdvancedChassisController::getThread() const {
    return task;
}

void AdvancedChassisController::stop() {
    LOG_INFO_S("AdvancedChassisController: Stopping");

    mode = none;
    doneLooping.store(true, std::memory_order_release);
    stopAfterSettled();
}

void AdvancedChassisController::setMaxVelocity(double imaxVelocity) {
    chassisModel->setMaxVelocity(imaxVelocity);
}

double AdvancedChassisController::getMaxVelocity() const {
    return chassisModel->getMaxVelocity();
}

std::shared_ptr<ChassisModel> AdvancedChassisController::getModel() {
    return chassisModel;
}

ChassisModel &AdvancedChassisController::model() {
    return *chassisModel;
}
