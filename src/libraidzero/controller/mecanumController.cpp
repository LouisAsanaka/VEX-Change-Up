#include "libraidzero/controller/mecanumController.hpp"
#include "main.h"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/chassis/model/xDriveModel.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/util/logging.hpp"
#include "pidController.hpp"
#include <memory>

MecanumController::MecanumController(
    TimeUtil itimeUtil,
    std::shared_ptr<OdomChassisController> odomChassisController,
    PIDController::Gains distanceGains, PIDController::Gains angleGains
) :
    logger{Logger::getDefaultLogger()},
    model{std::dynamic_pointer_cast<XDriveModel>(odomChassisController->getModel())},
    odometry{odomChassisController->getOdometry()},
    scales{odomChassisController->getChassisScales()},
    pair{odomChassisController->getGearsetRatioPair()},
    timeUtil(std::move(itimeUtil)),
    distancePid(std::make_unique<PIDController>(distanceGains)),
    anglePid(std::make_unique<PIDController>(angleGains)),
    chassisController{std::move(odomChassisController)}
{   
}

MecanumController::~MecanumController() {
    dtorCalled.store(true, std::memory_order_release);
    isDisabled();

    delete task;
}

void MecanumController::strafeToPose(Pose2d ipose) {
    targetPose = ipose;
    mode = ModeType::Strafing;
    isRunning.store(true, std::memory_order_release);
}

bool MecanumController::isSettled() {
    if (isDisabled()) {
        return true;
    }
    switch (mode) {
    case ModeType::Strafing:
        return distancePid->atSetpoint() && anglePid->atSetpoint();
    case ModeType::None:
        return true;
    }
}

bool MecanumController::waitUntilSettled(int itimeout) {
    // make it as large as possible, so effectively no timeout
    if (itimeout == 0) {
        itimeout = ~itimeout;
    }
    timeout = itimeout;
    auto rate = timeUtil.getRate();
    uint32_t now = pros::millis();
    bool settled = isSettled();
    bool timeLeft = (pros::millis() - now < timeout);
    while (!settled && timeLeft) {
        rate->delayUntil(10_ms);

        settled = isSettled();
        timeLeft = (pros::millis() - now < timeout);
    }
    this->flipDisable(true);
    model->stop();
    return settled;
}

void MecanumController::reset() {
    flipDisable(true);

    LOG_INFO_S("MecanumController: Waiting to reset");

    auto rate = timeUtil.getRate();
    while (isRunning.load(std::memory_order_acquire)) {
        rate->delayUntil(1_ms);
    }
    flipDisable(false);
}

void MecanumController::flipDisable(const bool iisDisabled) {
    LOG_INFO("MecanumController: flipDisable " + std::to_string(iisDisabled));
    disabled.store(iisDisabled, std::memory_order_release);
}

bool MecanumController::isDisabled() const {
    return disabled.load(std::memory_order_acquire);
}

void MecanumController::startThread() {
    if (!task) {
        task = new CrossplatformThread(trampoline, this, "MecanumController");
    }
}

CrossplatformThread *MecanumController::getThread() const {
    return task;
}

void MecanumController::trampoline(void *context) {
    if (context != nullptr) {
        static_cast<MecanumController *>(context)->loop();
    }
}

void MecanumController::loop() {
    LOG_INFO_S("Started MecanumController task.");

    auto rate = timeUtil.getRate();

    while (!dtorCalled.load(std::memory_order_acquire) && !task->notifyTake(0)) {
        if (isRunning.load(std::memory_order_acquire) && !isDisabled()) {
            switch (mode) {
            case ModeType::Strafing:
                LOG_INFO("MecanumController: Strafing to target pose: " + targetPose.toString());

                executeStrafe(targetPose, timeUtil.getRate());
                model->stop();

                LOG_INFO_S("MecanumController: Done strafing.");
                break;
            case ModeType::None:
                break;
            }
            isRunning.store(false, std::memory_order_release);
        }

        rate->delayUntil(20_ms);
    }

    LOG_INFO_S("Stopped MecanumController task.");
}

void MecanumController::executeStrafe(const Pose2d& target, 
    std::unique_ptr<AbstractRate> rate)
{
    auto targetTranslation = target.translation();
    double targetAngle = target.rotation().angle().convert(radian);

    std::string message = "MecanumController: Target Pose=" + target.toString();
    LOG_INFO(message);

    // Go from a distance x to ~0.0, meaning that drive is on the target
    distancePid->reset();
    distancePid->setSetpoint(0.0);
    distancePid->setOutputLimits(-1.0, 1.0);
    // Go from the current angle x to target angle
    anglePid->reset();
    anglePid->setSetpoint(targetAngle);
    anglePid->setOutputLimits(-1.0, 1.0);

    auto timer = timeUtil.getTimer();

    // Mark the first moment
    timer->getDt();
    while (!isSettled()) {
        auto currentPose = Pose2d::fromOdomState(odometry->getState());

        std::string message = "MecanumController: Odom Pose=" + currentPose.toString();
        LOG_INFO(message);

        // Find the direction the drive should move towards in global coordinates
        auto directionVector = targetTranslation - currentPose.translation();
        // Use the same vector to find the distance to the target
        double distance = directionVector.norm().convert(meter);

        // Should always be negated since setpoint is always 0, and distance
        // is always positive. The direction vector only needs the magnitude.
        double distanceOutput = -distancePid->calculate(distance);

        directionVector /= distance;
        std::cout << directionVector.norm().convert(meter) << " should be 1" << std::endl;

        directionVector *= distanceOutput;
        directionVector = directionVector.rotateBy(-currentPose.rotation());

        double angleOutput = -anglePid->calculate(
            currentPose.rotation().angle().convert(radian));

        std::cout << "Distance PID: " << distanceOutput << " | Angle PID: " << angleOutput << std::endl;

        model->xArcade(directionVector.x().convert(meter), 
            directionVector.y().convert(meter), angleOutput);

        rate->delayUntil(10_ms);
    }    
}
