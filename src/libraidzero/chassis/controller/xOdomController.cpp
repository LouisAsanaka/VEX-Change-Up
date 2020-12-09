#include <atomic>
#include <memory>
#include <utility>

#include "gui.hpp"
#include "libraidzero/chassis/controller/xOdomController.hpp"
#include "libraidzero/chassis/model/threeEncoderImuXDriveModel.hpp"
#include "libraidzero/geometry/pose2d.hpp"
#include "libraidzero/geometry/rotation2d.hpp"
#include "libraidzero/geometry/translation2d.hpp"
#include "libraidzero/util/mathUtil.hpp"
#include "libraidzero/util/plotter.hpp"
#include "okapi/api/odometry/odomMath.hpp"
#include "okapi/api/odometry/stateMode.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/util/timeUtil.hpp"

XOdomController::XOdomController(
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
    slewRate{std::move(islewRate)},
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
}

void XOdomController::driveForDistance(QLength idistance, int itimeout, 
    std::vector<AsyncAction> iactions) 
{
    parseTimeout(itimeout);

    distancePid->reset();
    anglePid->reset();
    distancePid->flipDisable(false);
    anglePid->flipDisable(false);
    turnPid->flipDisable(true);
    strafeDistancePid->flipDisable(true);
    strafeAnglePid->flipDisable(true);

    const double newTarget = idistance.convert(meter) * scales.straight * gearsetRatioPair.ratio;

    LOG_INFO("XOdomController: moving " + std::to_string(newTarget) + " motor ticks");

    distancePid->setTarget(newTarget);
    anglePid->setTarget(0);

    auto encStartVals = model->getSensorVals();
    std::valarray<std::int32_t> encVals;
    double distanceElapsed = 0.0;
    double angleChange = 0.0;

    const auto currentPose = Pose2d::fromOdomState(getState());
    const Translation2d targetTranslation = currentPose.translation() + 
        Translation2d{
            idistance * currentPose.rotation().cos(), 
            idistance * currentPose.rotation().sin()
        };
    double distanceError = 0.0;

    makeSettlableLoop(isDistanceSettled, itimeout, iactions, distanceError, {
        encVals = model->getSensorVals() - encStartVals;
        distanceElapsed = static_cast<double>((encVals[0] + encVals[1])) / 2.0;
        angleChange = static_cast<double>(encVals[0] - encVals[1]); // left - right

        distanceError = targetTranslation.distance(
            Pose2d::fromOdomState(getState()).translation()).abs().convert(meter);
        std::cout << "moveDistance error: " << distanceError << " m" << std::endl;

        distancePid->step(distanceElapsed);
        anglePid->step(angleChange);
        model->driveVectorVoltage(distancePid->getOutput(), anglePid->getOutput());
    });
    stopAfterSettled();
}

void XOdomController::driveToPoint(const Point& ipoint, bool ibackwards, 
    int itimeout, std::vector<AsyncAction> iactions) 
{
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

void XOdomController::turnAngle(QAngle iangle, TurnType iturnType, int itimeout, 
    std::vector<AsyncAction> iactions) 
{
    parseTimeout(itimeout);

    LOG_INFO("XOdomController: turning " + std::to_string(iangle.convert(degree)) +
            " degrees");

    turnPid->reset();
    turnPid->flipDisable(false);
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    strafeDistancePid->flipDisable(true);
    strafeAnglePid->flipDisable(true);

    const double newAngle =
        iangle.convert(degree) * scales.turn * gearsetRatioPair.ratio;

    LOG_INFO("XOdomController: turning " + std::to_string(newAngle) + " motor ticks");

    turnPid->setTarget(newAngle);

    auto encStartVals = model->getSensorVals();
    std::valarray<std::int32_t> encVals;
    double angleChange = 0.0;

    const QAngle targetAngle = Pose2d::fromOdomState(getState()).rotation().angle() + iangle;
    double angleError = 0.0;

    makeSettlableLoop(isAngleSettled, itimeout, iactions, angleError, {
        encVals = model->getSensorVals() - encStartVals;
        angleChange = (encVals[1] - encVals[0]) / 2.0; // right - left

        angleError = (targetAngle - Pose2d::fromOdomState(getState()).rotation().angle())
            .abs().convert(radian);

        turnPid->step(angleChange);

        switch (iturnType) {
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
    });
    stopAfterSettled();
}

void XOdomController::turnToAngle(QAngle iangle, TurnType iturnType, int itimeout, 
    std::vector<AsyncAction> iactions)
{
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

void XOdomController::turnToPoint(const Point& ipoint, int itimeout, 
    std::vector<AsyncAction> iactions) 
{
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

void XOdomController::strafeToPoint(const Point& ipoint, int itimeout, 
    std::vector<AsyncAction> iactions) 
{
    strafeToPose(
        Pose2d{
            Translation2d{ipoint.x, ipoint.y}, 
            Rotation2d{-getState().theta}
        }, 
        itimeout
    );
}

void XOdomController::updateStrafeToPose(
    const Translation2d& itargetTranslation, double& idistanceError
) {
    // Use cartesian to flip x & y axes since odom works in a
    // different frame
    Pose2d currentPose = Pose2d::fromOdomState(getState());

    std::string message = "XOdomController: Odom Pose=" + currentPose.toString();
    LOG_INFO(message);

    // Find the direction the drive should move towards in global coordinates
    auto directionVector = itargetTranslation - currentPose.translation();

    // Use the same vector to find the distance to the target
    double distance = directionVector.norm().convert(meter);

    // Write the absolute distance error
    idistanceError = std::abs(distance);

    // Should always be negated since setpoint is always 0, and distance
    // is always positive. The direction vector only needs the magnitude.
    double distanceOutput = slewRate->calculate(-strafeDistancePid->step(distance));

    // QAngle gyroRotation = -currentPose.rotation().angle();
    QAngle gyroRotation = -model->getHeading() * degree;

    // Normalize the vector & scale it by the PID output
    directionVector /= distance;
    directionVector *= distanceOutput;
    directionVector = directionVector.rotateBy(Rotation2d{-gyroRotation});

    // std::cout << "DirX: " << directionVector.x().convert(meter) << ", DirY:" << directionVector.y().convert(meter) << std::endl;

    double angleOutput = strafeAnglePid->step(
        gyroRotation.convert(radian)
    );
    //std::cout << gyroHeading.convert(radian) << " | " << strafeAnglePid->getError() << std::endl;

    // std::cout << "Distance PID: " << distanceOutput << " | Angle PID: " << angleOutput << std::endl;

    // Negate the angle output since xArcade takes + as clockwise
    model->xArcade(
        directionVector.x().convert(meter),
        directionVector.y().convert(meter), 
        -angleOutput
    );
}

void XOdomController::strafeToPose(const Pose2d& ipose, int itimeout, 
    std::vector<AsyncAction> iactions) 
{
    waitForOdomTask();

    parseTimeout(itimeout);

    LOG_INFO("XOdomController: strafing to " + ipose.toString());

    strafeDistancePid->reset();
    strafeDistancePid->flipDisable(false);
    strafeDistancePid->setTarget(0.0);
    strafeAnglePid->reset();
    strafeAnglePid->flipDisable(false);
    strafeAnglePid->setTarget(ipose.rotation().angle().convert(radian));
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    turnPid->flipDisable(true);

    auto targetTranslation = ipose.translation();
    double distanceError = 0.0;
    makeSettlableLoop(isStrafeSettled, itimeout, iactions, distanceError, {
        updateStrafeToPose(targetTranslation, distanceError);
    });
    stopAfterSettled();
}

void XOdomController::executeActions(
    double iunitsError, int itimeFromStart, 
    std::vector<AsyncAction>& iactions
) {
    if (!iactions.empty()) {
        const AsyncAction& nextAction = iactions.at(0);
        if (iunitsError < nextAction.unitsError ||
            (nextAction.timeFromStart != -1 && itimeFromStart > nextAction.timeFromStart)
        ) { // If the error is within the bounds or a time limit is exceeded
            nextAction.callback();
            iactions.erase(iactions.begin());
        }
    }
}

void XOdomController::setState(OdomState istate) {
    odometry->setState(istate);
}

void XOdomController::setPose(const Pose2d &ipose) {
    odometry->setState({
        ipose.translation().x(), ipose.translation().y(), 
        constrainAnglePi(-ipose.rotation().angle().convert(radian)) * radian
    }, StateMode::CARTESIAN);
}

OdomState XOdomController::getState() {
    return odometry->getState(StateMode::CARTESIAN);
}

void XOdomController::stop() {
    LOG_INFO_S("XOdomController: Stopping");

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

bool XOdomController::isDistanceSettled() {
    return distancePid->isSettled() && anglePid->isSettled();
}

bool XOdomController::isAngleSettled() {
    return turnPid->isSettled();
}

bool XOdomController::isStrafeSettled() {
    return strafeDistancePid->isSettled() && strafeAnglePid->isSettled();
}

void XOdomController::stopAfterSettled() {
    distancePid->flipDisable(true);
    anglePid->flipDisable(true);
    turnPid->flipDisable(true);
    strafeDistancePid->flipDisable(true);
    strafeAnglePid->flipDisable(true);
    model->stop();
}

void XOdomController::loop() {
    odomTaskRunning = true;

    LOG_INFO_S("Started XOdomController odometry task.");

    int i = 0;
    auto rate = timeUtil.getRate();
    while (!dtorCalled.load(std::memory_order_acquire) && (task->notifyTake(0) == 0U)) {
        odometry->step();
        if (i == 5) {
            auto currentPose = Pose2d::fromOdomState(getState());
            GUI::getInstance().setData({
                currentPose.translation().x(), currentPose.translation().y(), currentPose.rotation().angle()
            }, {0, 0, 0});
            i = 0;
        } else {
            ++i;
        }
        rate->delayUntil(10_ms);
    }
    odomTaskRunning = false;
    LOG_INFO_S("Stopped XOdomController odometry task.");
}
