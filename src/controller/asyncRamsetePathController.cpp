#include "controller/asyncRamsetePathController.hpp"
#include "main.h"
#include "util/miscUtil.hpp"
#include "controller/util/ramseteUtil.hpp"
#include "geometry/pose2d.hpp"
#include "geometry/rotation2d.hpp"
#include "kinematics/kinematics.hpp"
#include "trajectory/trajectory.hpp"

#include <algorithm>
#include <mutex>
#include <numeric>
#include <iostream>

AsyncRamsetePathController::AsyncRamsetePathController(
    const TimeUtil &itimeUtil,
    const PathfinderLimits &ilimits,
    const std::shared_ptr<OdomChassisController> &ichassis,
    const RamseteConstants &iramseteConstants)
    : logger(Logger::getDefaultLogger()),
      limits(ilimits),
      model(ichassis->getModel()),
      scales(ichassis->getChassisScales()),
      kinematics(Kinematics{ichassis->getChassisScales()}),
      pair(ichassis->getGearsetRatioPair()),
      timeUtil(itimeUtil),
      chassis(ichassis),
      ramseteController(std::make_shared<RamseteController>(
        iramseteConstants.b, iramseteConstants.zeta)) {
}

AsyncRamsetePathController::~AsyncRamsetePathController() {
    dtorCalled.store(true, std::memory_order_release);
    isDisabled();

    // Free paths before deleting the task
    std::scoped_lock lock(currentPathMutex);
    paths.clear();

    delete task;
}

void AsyncRamsetePathController::generatePath(std::initializer_list<PathfinderPoint> iwaypoints,
                                              const std::string &ipathId, bool storePath) {
    if (iwaypoints.size() == 0) {
        // No point in generating a path
        LOG_WARN_S(
            "AsyncRamsetePathController: Not generating a path because no waypoints were given.");
        return;
    }
    std::vector<Waypoint> points;
    points.reserve(iwaypoints.size());
    for (auto &point : iwaypoints) {
        points.push_back(
            Waypoint{
                point.x.convert(meter), 
                point.y.convert(meter), 
                point.theta.convert(radian)
            });
    }
    LOG_INFO_S("AsyncRamsetePathController: Preparing trajectory");
    TrajectoryPtr candidate(new TrajectoryCandidate, [](TrajectoryCandidate *c) {
        if (c->laptr) {
            free(c->laptr);
        }
        if (c->saptr) {
            free(c->saptr);
        }
        delete c;
    });
    pathfinder_prepare(points.data(),
                       static_cast<int>(points.size()),
                       FIT_HERMITE_CUBIC,
                       PATHFINDER_SAMPLES_FAST,
                       0.010,
                       limits.maxVel,
                       limits.maxAccel,
                       limits.maxJerk,
                       candidate.get());
    const int length = candidate->length;
    if (length < 0) {
        std::string message = "AsyncRamsetePathController: Length was negative. " +
                                getPathErrorMessage(points, ipathId, length);
        LOG_ERROR(message);
        throw std::runtime_error(message);
    }
    SegmentPtr trajectory(static_cast<Segment *>(malloc(length * sizeof(Segment))), free);
    if (trajectory == nullptr) {
        std::string message = "AsyncRamsetePathController: Could not allocate trajectory. " +
                                getPathErrorMessage(points, ipathId, length);
        LOG_ERROR(message);
        throw std::runtime_error(message);
    }
    LOG_INFO_S("AsyncRamsetePathController: Generating path");
    pathfinder_generate(candidate.get(), trajectory.get());

    // Free the old path before overwriting it
    forceRemovePath(ipathId);

    paths.emplace(ipathId, Trajectory::segmentToStates(trajectory, length));

    LOG_INFO("AsyncRamsetePathController: Completely done generating path " + ipathId);
    LOG_DEBUG("AsyncRamsetePathController: Path length: " + std::to_string(length));

    if (storePath) {
        internalStorePath(ipathId, trajectory, length);
    }
}

bool AsyncRamsetePathController::removePath(const std::string &ipathId) {
    if (!isDisabled() && isRunning.load(std::memory_order_acquire) && getTarget() == ipathId) {
        LOG_WARN("AsyncRamsetePathController: Attempted to remove currently running path " + ipathId);
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

std::vector<std::string> AsyncRamsetePathController::getPaths() {
    std::vector<std::string> keys;

    for (const auto &path : paths) {
        keys.push_back(path.first);
    }

    return keys;
}

void AsyncRamsetePathController::setTarget(std::string ipathId, bool resetState,
    bool ibackwards, bool imirrored
) {
    LOG_INFO("AsyncRamsetePathController: Set target to: " + ipathId + " (ibackwards=" +
            std::to_string(ibackwards) + ", imirrored=" + std::to_string(imirrored) + ")");
    currentPath = ipathId;
    shouldResetState.store(resetState, std::memory_order_release);
    direction.store(boolToSign(!ibackwards), std::memory_order_release);
    mirrored.store(imirrored, std::memory_order_release);
    isRunning.store(true, std::memory_order_release);
}

void AsyncRamsetePathController::controllerSet(std::string ivalue) {
    setTarget(ivalue);
}

std::string AsyncRamsetePathController::getTarget() {
    return currentPath;
}

bool AsyncRamsetePathController::waitUntilSettled(int itimeout) {
    // make it as large as possible, so effectively no timeout
    if (!itimeout) {
        itimeout = ~itimeout;
    }
    timeout = itimeout;
    auto rate = timeUtil.getRate();
    uint32_t now = pros::millis();
    bool settled = isSettled(), timeLeft = (pros::millis() - now < timeout);
    while (!settled && timeLeft) {
        rate->delayUntil(10_ms);

        settled = isSettled();
        timeLeft = (pros::millis() - now < timeout);
    }
    this->flipDisable(true);
    model->stop();
    return settled;
}

bool AsyncRamsetePathController::isSettled() {
    return isDisabled() || !isRunning.load(std::memory_order_acquire);
}

void AsyncRamsetePathController::reset() {
    // Interrupt executeSinglePath() by disabling the controller
    flipDisable(true);

    LOG_INFO_S("AsyncRamsetePathController: Waiting to reset");

    auto rate = timeUtil.getRate();
    while (isRunning.load(std::memory_order_acquire)) {
        rate->delayUntil(1_ms);
    }
    flipDisable(false);
}

void AsyncRamsetePathController::flipDisable() {
    flipDisable(!disabled.load(std::memory_order_acquire));
}

void AsyncRamsetePathController::flipDisable(const bool iisDisabled) {
    LOG_INFO("AsyncRamsetePathController: flipDisable " + std::to_string(iisDisabled));
    disabled.store(iisDisabled, std::memory_order_release);
    // loop() will stop the chassis when executeSinglePath() is done
    // the default implementation of executeSinglePath() breaks when disabled
}

bool AsyncRamsetePathController::isDisabled() const {
    return disabled.load(std::memory_order_acquire);
}

void AsyncRamsetePathController::tarePosition() {
}

void AsyncRamsetePathController::startThread() {
    if (!task) {
        task = new CrossplatformThread(trampoline, this, "AsyncRamsetePathController");
    }
}

CrossplatformThread *AsyncRamsetePathController::getThread() const {
    return task;
}

void AsyncRamsetePathController::trampoline(void *context) {
    if (context) {
        static_cast<AsyncRamsetePathController *>(context)->loop();
    }
}

void AsyncRamsetePathController::loop() {
    LOG_INFO_S("Started AsyncRamsetePathController task.");

    auto rate = timeUtil.getRate();

    while (!dtorCalled.load(std::memory_order_acquire) && !task->notifyTake(0)) {
        if (isRunning.load(std::memory_order_acquire) && !isDisabled()) {
            LOG_INFO("AsyncRamsetePathController: Running with path: " + currentPath);

            auto pair = paths.find(currentPath);
            if (pair == paths.end()) {
                LOG_WARN("AsyncRamsetePathController: Target was set to non-existent path with name: " +
                        currentPath);
            } else {
                LOG_DEBUG("AsyncRamsetePathController: Path length is " +
                        std::to_string(pair->second.getLength()));

                executeSinglePath(pair->second, timeUtil.getRate());
                model->stop();

                LOG_INFO_S("AsyncRamsetePathController: Done moving");
            }

            isRunning.store(false, std::memory_order_release);
        }

        rate->delayUntil(10_ms);
    }

    LOG_INFO_S("Stopped AsyncRamsetePathController task.");
}

void AsyncRamsetePathController::executeSinglePath(const Trajectory& trajectory,
                                                   std::unique_ptr<AbstractRate> rate) {
    const int reversed = direction.load(std::memory_order_acquire);
    const bool followMirrored = mirrored.load(std::memory_order_acquire);

    if (shouldResetState.load(std::memory_order_acquire)) {
        std::scoped_lock lock(currentPathMutex);
        const Pose2d& startingPose = trajectory.getStates()[0].pose;
        chassis->setState(OdomState{
            startingPose.translation().x(), startingPose.translation().y(),
            startingPose.rotation().angle()
        });
        std::string message = "AsyncRamsetePathController: Set " + startingPose.toString() + " as initial state.";
        LOG_INFO(message);
    }

    auto timer = timeUtil.getTimer();

    // Mark the first moment
    timer->getDt();
    while (!isSettled()) {
        // This mutex is used to combat an edge case of an edge case
        // if a running path is asked to be removed at the moment this loop is executing
        std::scoped_lock lock(currentPathMutex);

        Trajectory::State sampled = trajectory.sample(timer->getDtFromStart().convert(second));

        auto wheelSpeeds = kinematics.toWheelSpeeds(
            ramseteController->calculate(
                Pose2d::fromOdomState(chassis->getState()),
                sampled.pose, sampled.vel * mps, sampled.angularVel * radps
            )
        );

        const auto leftRPM = convertLinearToRotational(wheelSpeeds.left).convert(rpm);
        const auto rightRPM = convertLinearToRotational(wheelSpeeds.right).convert(rpm);

        const double leftSpeed = leftRPM / toUnderlyingType(pair.internalGearset) * reversed;
        const double rightSpeed = rightRPM / toUnderlyingType(pair.internalGearset) * reversed;

        if (followMirrored) {
            model->left(rightSpeed);
            model->right(leftSpeed);
        } else {
            model->left(leftSpeed);
            model->right(rightSpeed);
        }

        // Unlock before the delay to be nice to other tasks
        currentPathMutex.unlock();

        rate->delayUntil(10_ms);
    }
}

std::string AsyncRamsetePathController::getPathErrorMessage(const std::vector<Waypoint> &points,
                                                            const std::string &ipathId,
                                                            int length) {
    auto pointToString = [](Waypoint point) {
        return "PathfinderPoint{x=" + std::to_string(point.x) + ", y=" + std::to_string(point.y) +
            ", theta=" + std::to_string(point.angle) + "}";
    };

    return "The path (id " + ipathId + ", length " + std::to_string(length) +
            ") is impossible with waypoints: " +
            std::accumulate(std::next(points.begin()),
                            points.end(),
                            pointToString(points.at(0)),
                            [&](std::string a, Waypoint b) { return a + ", " + pointToString(b); });
}

QAngularSpeed AsyncRamsetePathController::convertLinearToRotational(QSpeed linear) const {
    return (linear * (360_deg / (scales.wheelDiameter * 1_pi))) * pair.ratio;
}

int AsyncRamsetePathController::getTrajectoryLength(const Trajectory &trajectory) {
    std::scoped_lock lock(currentPathMutex);
    return trajectory.getLength();
}

const Pose2d& AsyncRamsetePathController::getInitialPose(const Trajectory &trajectory) {
    std::scoped_lock lock(currentPathMutex);
    return trajectory.getStates()[0].pose;
}

void AsyncRamsetePathController::loadPath(const std::string &ipathId) {
    std::string filePath = makeFilePath("paths", ipathId + ".csv");
    FILE *pathFile = fopen(filePath.c_str(), "r");

    // Make sure we can open the file successfully
    if (pathFile == NULL) {
        LOG_WARN("AsyncRamsetePathController: Couldn't open file " + filePath + " for reading");
        if (pathFile != NULL) {
            fclose(pathFile);
        }
        return;
    }
    internalLoadPath(pathFile, ipathId);

    fclose(pathFile);
}

void AsyncRamsetePathController::internalStorePath(const std::string &ipathId,
                                                   SegmentPtr& trajectory, int length) {
    std::string filePath = makeFilePath("paths", ipathId + ".csv");
    FILE *pathFile = fopen(filePath.c_str(), "w");

    // Make sure we can open the file successfully
    if (pathFile == NULL) {
        LOG_WARN("AsyncRamsetePathController: Couldn't open file " + filePath + " for writing");
        if (pathFile != NULL) {
            fclose(pathFile);
        }
        return;
    }

    pathfinder_serialize_csv(pathFile, trajectory.get(), length);

    fclose(pathFile);
}

void AsyncRamsetePathController::internalLoadPath(FILE *pathFile,
                                                  const std::string &ipathId) {
    // Count lines in file, remove one for headers
    int count = 0;
    for (int c = getc(pathFile); c != EOF; c = getc(pathFile)) {
        if (c == '\n') {
            ++count;
        }
    }
    --count;
    rewind(pathFile);

    // Allocate memory
    SegmentPtr trajectory((Segment *)malloc(sizeof(Segment) * count), free);

    pathfinder_deserialize_csv(pathFile, trajectory.get());

    // Remove the old path if it exists
    forceRemovePath(ipathId);
    paths.emplace(ipathId, Trajectory::segmentToStates(trajectory, count));
}

std::string AsyncRamsetePathController::makeFilePath(const std::string &directory,
                                                     const std::string &filename) {
    std::string path(directory);

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

    return path;
}

void AsyncRamsetePathController::forceRemovePath(const std::string &ipathId) {
    if (!removePath(ipathId)) {
        LOG_WARN("AsyncRamsetePathController: Disabling controller to remove path " + ipathId);
        flipDisable(true);
        removePath(ipathId);
    }
}
