/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "libraidzero/controller/advancedOdomChassisController.hpp"
#include "libraidzero/controller/advancedChassisController.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include <cmath>
#include <utility>

using namespace okapi;

AdvancedOdomChassisController::AdvancedOdomChassisController(
    const TimeUtil &itimeUtil,
    std::shared_ptr<Odometry> iodometry,
    std::shared_ptr<ChassisController> icontroller,
    const StateMode &imode,
    const QLength imoveThreshold,
    const QAngle iturnThreshold,
    std::shared_ptr<Logger> ilogger)
    : OdomChassisController(itimeUtil, std::move(iodometry), imode, imoveThreshold, iturnThreshold),
        logger(std::move(ilogger)),
        controller(std::static_pointer_cast<AdvancedChassisController>(icontroller)) {
}

void AdvancedOdomChassisController::waitForOdomTask() {
    if (odomTaskRunning) {
        // Early exit to save calling getRate
        return;
    }

    auto rate = timeUtil.getRate();
    while (!odomTaskRunning) {
        LOG_INFO_S("AdvancedOdomChassisController: Waiting for odometry task to start.");
        rate->delayUntil(10);
    }
}

void AdvancedOdomChassisController::driveToPoint(const Point &ipoint,
                                                 const bool ibackwards,
                                                 const QLength &ioffset) {
    waitForOdomTask();

    auto pair = OdomMath::computeDistanceAndAngleToPoint(
        ipoint.inFT(defaultStateMode), odom->getState(StateMode::FRAME_TRANSFORMATION));
    auto length = pair.first;
    auto angle = pair.second;

    if (ibackwards) {
        length *= -1;
        angle += 180_deg;
    }

    LOG_INFO("AdvancedOdomChassisController: Computed length of " +
           std::to_string(length.convert(meter)) + " meters and angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("AdvancedOdomChassisController: Turning " + std::to_string(angle.convert(degree)) +
             " degrees");
        controller->turnAngle(angle);
    }

    if ((length - ioffset).abs() > moveThreshold) {
        LOG_INFO("AdvancedOdomChassisController: Driving " +
             std::to_string((length - ioffset).convert(meter)) + " meters");
        controller->moveDistance(length - ioffset);
    }
}

void AdvancedOdomChassisController::turnToPoint(const Point &ipoint) {
    waitForOdomTask();

    const auto angle = OdomMath::computeAngleToPoint(ipoint.inFT(defaultStateMode),
                                                     odom->getState(StateMode::FRAME_TRANSFORMATION));

    LOG_INFO("AdvancedOdomChassisController: Computed angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("AdvancedOdomChassisController: Turning " + std::to_string(angle.convert(degree)) +
             " degrees");
        controller->turnAngle(angle);
    }
}

void AdvancedOdomChassisController::turnToAngle(const QAngle &iangle) {
    waitForOdomTask();

    const auto angle = iangle - odom->getState(StateMode::FRAME_TRANSFORMATION).theta;

    LOG_INFO("AdvancedOdomChassisController: Computed angle of " +
           std::to_string(angle.convert(degree)) + " degrees");

    if (angle.abs() > turnThreshold) {
        LOG_INFO("AdvancedOdomChassisController: Turning " + std::to_string(angle.convert(degree)) +
             " degrees");
        controller->turnAngle(angle);
    }
}

void AdvancedOdomChassisController::moveDistance(QLength itarget) {
    controller->moveDistance(itarget);
}

void AdvancedOdomChassisController::moveRaw(double itarget) {
    controller->moveRaw(itarget);
}

void AdvancedOdomChassisController::moveDistanceAsync(QLength itarget) {
    controller->moveDistanceAsync(itarget);
}

void AdvancedOdomChassisController::moveRawAsync(double itarget) {
    controller->moveRawAsync(itarget);
}

void AdvancedOdomChassisController::turnAngle(QAngle idegTarget) {
    controller->turnAngle(idegTarget);
}

void AdvancedOdomChassisController::turnAngle(QAngle idegTarget, TurnType iturnType) {
    controller->turnAngle(idegTarget, iturnType);
}

void AdvancedOdomChassisController::turnRaw(double idegTarget) {
    controller->turnRaw(idegTarget);
}

void AdvancedOdomChassisController::turnRaw(double idegTarget, TurnType iturnType) {
    controller->turnRaw(idegTarget, iturnType);
}

void AdvancedOdomChassisController::turnAngleAsync(QAngle idegTarget) {
    controller->turnAngleAsync(idegTarget);
}

void AdvancedOdomChassisController::turnAngleAsync(QAngle idegTarget, TurnType iturnType) {
    controller->turnAngleAsync(idegTarget, iturnType);
}

void AdvancedOdomChassisController::turnRawAsync(double idegTarget) {
    controller->turnRawAsync(idegTarget);
}

void AdvancedOdomChassisController::turnRawAsync(double idegTarget, TurnType iturnType) {
    controller->turnRawAsync(idegTarget, iturnType);
}

void AdvancedOdomChassisController::setTurnsMirrored(bool ishouldMirror) {
    controller->setTurnsMirrored(ishouldMirror);
}

bool AdvancedOdomChassisController::isSettled() {
    return controller->isSettled();
}

void AdvancedOdomChassisController::waitUntilSettled() {
    controller->waitUntilSettled();
}

void AdvancedOdomChassisController::stop() {
    controller->stop();
}

void AdvancedOdomChassisController::setMaxVelocity(double imaxVelocity) {
    controller->setMaxVelocity(imaxVelocity);
}

double AdvancedOdomChassisController::getMaxVelocity() const {
    return controller->getMaxVelocity();
}

ChassisScales AdvancedOdomChassisController::getChassisScales() const {
    return controller->getChassisScales();
}

AbstractMotor::GearsetRatioPair AdvancedOdomChassisController::getGearsetRatioPair() const {
    return controller->getGearsetRatioPair();
}

std::shared_ptr<ChassisModel> AdvancedOdomChassisController::getModel() {
    return controller->getModel();
}

ChassisModel &AdvancedOdomChassisController::model() {
    return controller->model();
}

std::shared_ptr<ChassisController> AdvancedOdomChassisController::getChassisController() {
    return controller;
}

ChassisController &AdvancedOdomChassisController::chassisController() {
    return *controller;
}
