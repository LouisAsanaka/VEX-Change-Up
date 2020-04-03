/*
 * @author Ryan Benasutti, WPI
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#include "libraidzero/controller/asyncAdvancedProfileController.hpp"
#include "libraidzero/kinematics/chassisSpeeds.hpp"
#include "libraidzero/planner/profilePlanner.hpp"
#include "libraidzero/planner/profileStructs.hpp"
#include "okapi/api/odometry/point.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include <algorithm>
#include <iostream>
#include <mutex>
#include <numeric>

AsyncAdvancedProfileController::AsyncAdvancedProfileController(
    const TimeUtil &itimeUtil,
    const planner::PlannerConfig &iconfig,
    const std::shared_ptr<ChassisModel> &imodel,
    const ChassisScales &iscales,
    const AbstractMotor::GearsetRatioPair &ipair,
    const std::shared_ptr<Logger> &ilogger)
    : logger(ilogger),
      config(iconfig),
      model(imodel),
      scales(iscales),
      kinematics(iscales),
      pair(ipair),
      timeUtil(itimeUtil) {
    if (ipair.ratio == 0) {
        std::string msg("AsyncAdvancedProfileController: The gear ratio cannot be zero! Check if you are "
                        "using integer division.");
        LOG_ERROR(msg);
        throw std::invalid_argument(msg);
    }
}

AsyncAdvancedProfileController::~AsyncAdvancedProfileController() {
    dtorCalled.store(true, std::memory_order_release);
    isDisabled();

    // Free paths before deleting the task
    std::scoped_lock lock(currentPathMutex);
    paths.clear();

    delete task;
}

void AsyncAdvancedProfileController::generatePath(std::initializer_list<planner::UnitableWaypoint> iwaypoints,
                                                  const std::string &ipathId) {
    generatePath(iwaypoints, ipathId, config);
}

void AsyncAdvancedProfileController::generatePath(std::initializer_list<planner::UnitableWaypoint> iwaypoints,
                                                  const std::string &ipathId,
                                                  const planner::PlannerConfig& iconfig) {
    if (iwaypoints.size() == 0) {
        // No point in generating a path
        LOG_WARN_S(
            "AsyncAdvancedProfileController: Not generating a path because no waypoints were given.");
        return;
    }

    std::vector<planner::Waypoint> points;
    points.reserve(iwaypoints.size());
    for (auto &point : iwaypoints) {
        if (point.angle.has_value()) {
            points.emplace_back(
                point.x.convert(meter), 
                point.y.convert(meter), 
                point.angle.value().convert(radian)
            );
        } else {
            points.emplace_back(
                point.x.convert(meter), 
                point.y.convert(meter)
            );
        }
    }

    LOG_INFO_S("AsyncAdvancedProfileController: Generating path");

    // Free the old path before overwriting it
    forceRemovePath(ipathId);

    paths[ipathId] = planner::ProfilePlanner::generatePath(
        points, iconfig
    );
   
    LOG_INFO("AsyncAdvancedProfileController: Completely done generating path " + ipathId);
    LOG_DEBUG("AsyncAdvancedProfileController: Path length: " + std::to_string(paths[ipathId].pathPoints.size()));
}

bool AsyncAdvancedProfileController::removePath(const std::string &ipathId) {
    if (!isDisabled() && isRunning.load(std::memory_order_acquire) && getTarget() == ipathId) {
        LOG_WARN("AsyncAdvancedProfileController: Attempted to remove currently running path " + ipathId);
        return false;
    }

    std::scoped_lock lock(currentPathMutex);

    auto oldPath = paths.find(ipathId);
    if (oldPath != paths.end()) {
        paths.erase(ipathId);
    }

    // A return value of true provides no feedback about whether the path was actually removed but
    // instead tells us that the path does not exist at this moment
    return true;
}

std::vector<std::string> AsyncAdvancedProfileController::getPaths() {
    std::vector<std::string> keys;

    for (const auto &path : paths) {
        keys.push_back(path.first);
    }

    return keys;
}

void AsyncAdvancedProfileController::setTarget(std::string ipathId) {
    setTarget(ipathId, false);
}

void AsyncAdvancedProfileController::setTarget(std::string ipathId,
                                               const bool ibackwards,
                                               const bool imirrored) {
    LOG_INFO("AsyncAdvancedProfileController: Set target to: " + ipathId + " (ibackwards=" +
           std::to_string(ibackwards) + ", imirrored=" + std::to_string(imirrored) + ")");

    currentPath = ipathId;
    direction.store(boolToSign(!ibackwards), std::memory_order_release);
    mirrored.store(imirrored, std::memory_order_release);
    isRunning.store(true, std::memory_order_release);
}

void AsyncAdvancedProfileController::controllerSet(std::string ivalue) {
    setTarget(ivalue);
}

std::string AsyncAdvancedProfileController::getTarget() {
    return currentPath;
}

std::string AsyncAdvancedProfileController::getProcessValue() const {
    return currentPath;
}

void AsyncAdvancedProfileController::loop() {
    LOG_INFO_S("Started AsyncAdvancedProfileController task.");

    auto rate = timeUtil.getRate();

    while (!dtorCalled.load(std::memory_order_acquire) && !task->notifyTake(0)) {
        if (isRunning.load(std::memory_order_acquire) && !isDisabled()) {
            LOG_INFO("AsyncAdvancedProfileController: Running with path: " + currentPath);

            auto path = paths.find(currentPath);
            if (path == paths.end()) {
                LOG_WARN("AsyncAdvancedProfileController: Target was set to non-existent path with name: " +
                 currentPath);
            } else {
                LOG_DEBUG("AsyncAdvancedProfileController: Path length is " +
                          std::to_string(path->second.pathPoints.size()));

                executeSinglePath(path->second, timeUtil.getRate());

                // Stop the chassis after the path because:
                // 1. We only support an exit velocity of zero
                // 2. Because of (1), we should make sure the system is stopped
                model->stop();

                LOG_INFO_S("AsyncAdvancedProfileController: Done moving");
            }

            isRunning.store(false, std::memory_order_release);
        }

        rate->delayUntil(10_ms);
    }

    LOG_INFO_S("Stopped AsyncAdvancedProfileController task.");
}

void AsyncAdvancedProfileController::executeSinglePath(const planner::MotionProfile &profile,
                                                       std::unique_ptr<AbstractRate> rate) {
    const int reversed = direction.load(std::memory_order_acquire);
    const bool followMirrored = mirrored.load(std::memory_order_acquire);
    const int pathLength = getProfileLength(profile);

    for (int i = 0; i < pathLength - 1 && !isDisabled(); ++i) {
        // This mutex is used to combat an edge case of an edge case
        // if a running path is asked to be removed at the moment this loop is executing
        std::scoped_lock lock(currentPathMutex);

        const auto segDT = profile.pathPoints[i + 1].time * second;
        const auto linearSpeed = profile.pathPoints[i].velocity * mps;
        const double deltaAngle = (profile.pathPoints[i + 1].angle - profile.pathPoints[i].angle) 
            * M_PI / 180;
        const auto angularSpeed = deltaAngle * radian / segDT;
        const auto wheelSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds{
            linearSpeed, 0_mps, angularSpeed
        });
        const auto leftRPM = convertLinearToRotational(wheelSpeeds.left).convert(rpm);
        const auto rightRPM = convertLinearToRotational(wheelSpeeds.right).convert(rpm);

        const double rightSpeed = rightRPM / toUnderlyingType(pair.internalGearset) * reversed;
        const double leftSpeed = leftRPM / toUnderlyingType(pair.internalGearset) * reversed;
        if (followMirrored) {
            model->left(rightSpeed);
            model->right(leftSpeed);
        } else {
            model->left(leftSpeed);
            model->right(rightSpeed);
        }

        // Unlock before the delay to be nice to other tasks
        currentPathMutex.unlock();

        rate->delayUntil(segDT);
    }
}

int AsyncAdvancedProfileController::getProfileLength(const planner::MotionProfile &profile) {
    std::scoped_lock lock(currentPathMutex);
    return profile.pathPoints.size();
}

QAngularSpeed AsyncAdvancedProfileController::convertLinearToRotational(QSpeed linear) const {
    return (linear * (360_deg / (scales.wheelDiameter * 1_pi))) * pair.ratio;
}

void AsyncAdvancedProfileController::trampoline(void *context) {
    if (context) {
        static_cast<AsyncAdvancedProfileController *>(context)->loop();
    }
}

void AsyncAdvancedProfileController::waitUntilSettled() {
    LOG_INFO_S("AsyncAdvancedProfileController: Waiting to settle");

    auto rate = timeUtil.getRate();
    while (!isSettled()) {
        rate->delayUntil(10_ms);
    }

    LOG_INFO_S("AsyncAdvancedProfileController: Done waiting to settle");
}

void AsyncAdvancedProfileController::moveTo(std::initializer_list<planner::UnitableWaypoint> iwaypoints,
                                            bool ibackwards,
                                            bool imirrored) {
    moveTo(iwaypoints, config, ibackwards, imirrored);
}

void AsyncAdvancedProfileController::moveTo(std::initializer_list<planner::UnitableWaypoint> iwaypoints,
                                            const planner::PlannerConfig& iconfig,
                                            const bool ibackwards,
                                            const bool imirrored) {
    static int moveToCount = 0;
    std::string name = "__moveTo" + std::to_string(moveToCount++);
    generatePath(iwaypoints, name, iconfig);
    setTarget(name, ibackwards, imirrored);
    waitUntilSettled();
    forceRemovePath(name);
}

planner::UnitableWaypoint AsyncAdvancedProfileController::getError() const {
    return planner::UnitableWaypoint{0_m, 0_m, 0_deg};
}

bool AsyncAdvancedProfileController::isSettled() {
    return isDisabled() || !isRunning.load(std::memory_order_acquire);
}

void AsyncAdvancedProfileController::reset() {
    // Interrupt executeSinglePath() by disabling the controller
    flipDisable(true);

    LOG_INFO_S("AsyncAdvancedProfileController: Waiting to reset");

    auto rate = timeUtil.getRate();
    while (isRunning.load(std::memory_order_acquire)) {
        rate->delayUntil(1_ms);
    }

    flipDisable(false);
}

void AsyncAdvancedProfileController::flipDisable() {
    flipDisable(!disabled.load(std::memory_order_acquire));
}

void AsyncAdvancedProfileController::flipDisable(const bool iisDisabled) {
    LOG_INFO("AsyncAdvancedProfileController: flipDisable " + std::to_string(iisDisabled));
    disabled.store(iisDisabled, std::memory_order_release);
    // loop() will stop the chassis when executeSinglePath() is done
    // the default implementation of executeSinglePath() breaks when disabled
}

bool AsyncAdvancedProfileController::isDisabled() const {
    return disabled.load(std::memory_order_acquire);
}

void AsyncAdvancedProfileController::tarePosition() {
}

void AsyncAdvancedProfileController::setMaxVelocity(std::int32_t) {
}

void AsyncAdvancedProfileController::startThread() {
    if (!task) {
        task = new CrossplatformThread(trampoline, this, "AsyncAdvancedProfileController");
    }
}

CrossplatformThread *AsyncAdvancedProfileController::getThread() const {
    return task;
}

void AsyncAdvancedProfileController::storePath(const std::string &idirectory,
                                             const std::string &ipathId) {
    /*std::string leftFilePath = makeFilePath(idirectory, ipathId + ".left.csv");
    std::string rightFilePath = makeFilePath(idirectory, ipathId + ".right.csv");
    FILE *leftPathFile = fopen(leftFilePath.c_str(), "w");
    FILE *rightPathFile = fopen(rightFilePath.c_str(), "w");

    // Make sure we can open the file successfully
    if (leftPathFile == NULL) {
        LOG_WARN("AsyncAdvancedProfileController: Couldn't open file " + leftFilePath + " for writing");
        if (rightPathFile != NULL) {
            fclose(rightPathFile);
        }
        return;
    }
    if (rightPathFile == NULL) {
        LOG_WARN("AsyncAdvancedProfileController: Couldn't open file " + rightFilePath + " for writing");
        fclose(leftPathFile);
        return;
    }

    internalStorePath(leftPathFile, rightPathFile, ipathId);

    fclose(leftPathFile);
    fclose(rightPathFile);*/
}

void AsyncAdvancedProfileController::loadPath(const std::string &idirectory,
                                                                                        const std::string &ipathId) {
    /*std::string leftFilePath = makeFilePath(idirectory, ipathId + ".left.csv");
    std::string rightFilePath = makeFilePath(idirectory, ipathId + ".right.csv");
    FILE *leftPathFile = fopen(leftFilePath.c_str(), "r");
    FILE *rightPathFile = fopen(rightFilePath.c_str(), "r");

    // Make sure we can open the file successfully
    if (leftPathFile == NULL) {
        LOG_WARN("AsyncAdvancedProfileController: Couldn't open file " + leftFilePath + " for reading");
        if (rightPathFile != NULL) {
            fclose(rightPathFile);
        }
        return;
    }
    if (rightPathFile == NULL) {
        LOG_WARN("AsyncAdvancedProfileController: Couldn't open file " + rightFilePath + " for reading");
        fclose(leftPathFile);
        return;
    }

    internalLoadPath(leftPathFile, rightPathFile, ipathId);

    fclose(leftPathFile);
    fclose(rightPathFile);*/
}

void AsyncAdvancedProfileController::internalStorePath(FILE *leftPathFile,
                                                     FILE *rightPathFile,
                                                     const std::string &ipathId) {
    /*auto pathData = this->paths.find(ipathId);

    // Make sure path exists
    if (pathData == paths.end()) {
        LOG_WARN("AsyncAdvancedProfileController: Controller was asked to serialize non-existent path " +
             ipathId);
        // Do nothing- can't serialize nonexistent path
    } else {
        int len = pathData->second.length;

        // Serialize paths
        pathfinder_serialize_csv(leftPathFile, pathData->second.left.get(), len);
        pathfinder_serialize_csv(rightPathFile, pathData->second.right.get(), len);
    }*/
}

void AsyncAdvancedProfileController::internalLoadPath(FILE *leftPathFile,
                                                      FILE *rightPathFile,
                                                      const std::string &ipathId) {
    /*
    // Count lines in file, remove one for headers
    int count = 0;
    for (int c = getc(leftPathFile); c != EOF; c = getc(leftPathFile)) {
        if (c == '\n') {
            ++count;
        }
    }
    --count;
    rewind(leftPathFile);

    // Allocate memory
    SegmentPtr leftTrajectory((Segment *)malloc(sizeof(Segment) * count), free);
    SegmentPtr rightTrajectory((Segment *)malloc(sizeof(Segment) * count), free);

    pathfinder_deserialize_csv(leftPathFile, leftTrajectory.get());
    pathfinder_deserialize_csv(rightPathFile, rightTrajectory.get());

    // Remove the old path if it exists
    forceRemovePath(ipathId);
    paths.emplace(ipathId,
                  TrajectoryPair{std::move(leftTrajectory), std::move(rightTrajectory), count});
    */
}

std::string AsyncAdvancedProfileController::makeFilePath(const std::string &directory,
                                                       const std::string &filename) {
    /*std::string path(directory);

    // Checks first substring
    if (path.rfind("/usd", 0) == std::string::npos) {
        if (path.rfind("usd", 0) != std::string::npos) {
            // There's a usd, but no beginning slash
            path.insert(0, "/"); // We just need a slash
        } else {               // There's nothing at all
            if (path.front() == '/') {
                // Don't double up on slashes
                path.insert(0, "/usd");
            } else {
                path.insert(0, "/usd/");
            }
        }
    }

    // Add trailing slash if there isn't one
    if (path.back() != '/') {
        path.append("/");
    }
    std::string filenameCopy(filename);
    // Remove restricted characters from filename
    static const std::string illegalChars = "\\/:?*\"<>|";
    for (auto it = filenameCopy.begin(); it < filenameCopy.end(); it++) {
        if (illegalChars.rfind(*it) != std::string::npos) {
            it = filenameCopy.erase(it);
        }
    }

    path.append(filenameCopy);

    return path;*/
}

void AsyncAdvancedProfileController::forceRemovePath(const std::string &ipathId) {
    if (!removePath(ipathId)) {
        LOG_WARN("AsyncAdvancedProfileController: Disabling controller to remove path " + ipathId);
        flipDisable(true);
        removePath(ipathId);
    }
}
