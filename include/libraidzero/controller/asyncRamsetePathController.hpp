#pragma once

#include "libraidzero/controller/ramseteController.hpp"
#include "libraidzero/controller/util/ramseteUtil.hpp"
#include "libraidzero/kinematics/kinematics.hpp"
#include "libraidzero/trajectory/trajectory.hpp"
#include <vector>
#include <map>
#include <cstdio>

class AsyncRamsetePathController {
public:
    AsyncRamsetePathController(const TimeUtil &itimeUtil,
                               const PathfinderLimits &ilimits,
                               const std::shared_ptr<OdomChassisController> &ichassis,
                               const RamseteConstants &iramseteConstants = RamseteConstants{});

    AsyncRamsetePathController(AsyncRamsetePathController &&other) = delete;
    AsyncRamsetePathController &operator=(AsyncRamsetePathController &&other) = delete;
  
    ~AsyncRamsetePathController();
  
    /**
     * Generates a path which intersects the given waypoints and saves it internally with a key of
     * pathId. Call `executePath()` with the same pathId to run it.
     *
     * If the waypoints form a path which is impossible to achieve, an instance of
     * `std::runtime_error` is thrown (and an error is logged) which describes the waypoints. If there
     * are no waypoints, no path is generated.
     *
     * @param iwaypoints The waypoints to hit on the path.
     * @param ipathId A unique identifier to save the path with.
     * @param storePath Whether to store the path to a CSV file.
     */
    void generatePath(std::initializer_list<PathfinderPoint> iwaypoints, const std::string &ipathId,
                      bool storePath = false);
  
    /**
     * Removes a path and frees the memory it used. This function returns true if the path was either
     * deleted or didn't exist in the first place. It returns false if the path could not be removed
     * because it is running.
     *
     * @param ipathId A unique identifier for the path, previously passed to `generatePath()`
     * @return True if the path no longer exists
     */
    bool removePath(const std::string &ipathId);
  
    /**
     * Gets the identifiers of all paths saved in this `AsyncRamsetePathController`.
     *
     * @return The identifiers of all paths
     */
    std::vector<std::string> getPaths();
  
    /**
     * Executes a path with the given ID. If there is no path matching the ID, the method will
     * return. Any targets set while a path is being followed will be ignored.
     *
     * @param ipathId A unique identifier for the path, previously passed to `generatePath()`.
     * @param resetState Whether to reset the odometry state to the starting position.
     * @param ibackwards Whether to follow the profile backwards.
     * @param imirrored Whether to follow the profile mirrored.
     */
    void setTarget(std::string ipathId, bool resetState = false, bool ibackwards = false, bool imirrored = false);
  
    /**
     * Writes the value of the controller output. This method might be automatically called in another
     * thread by the controller. This just calls `setTarget()`.
     */
    void controllerSet(std::string ivalue);
  
    /**
     * Gets the last set target, or the default target if none was set.
     *
     * @return the last target
     */
    std::string getTarget();

    /**
	 * Delays program execution until the robot is at its target.
	 *
	 * @param timeout
	 *        The maximum delay time for this function. Passing 0 will allow the
	 *        function to delay infinitely.
	 * @return true for normal exit, false for timeout
	 */
    bool waitUntilSettled(int itimeout = 0);
  
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
     * Resets the controller so it can start from 0 again properly. Keeps configuration from
     * before. This implementation also stops movement.
     */
    void reset();
  
    /**
     * Changes whether the controller is off or on. Turning the controller on after it was off will
     * NOT cause the controller to move to its last set target.
     */
    void flipDisable();
  
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
     * Sets the "absolute" zero position of the controller to its current position.
     *
     * This implementation does nothing because the API always requires the starting position to be
     * specified.
     */
    void tarePosition();
  
    /**
     * Starts the internal thread. This should not be called by normal users. This method is called
     * by the `AsyncRamsetePathControllerBuilder` when making a new instance of this class.
     */
    void startThread();
  
    /**
     * @return The underlying thread handle.
     */
    CrossplatformThread *getThread() const;

    /**
     * Loads a path from a directory on the SD card containing path CSV files. `/usd/` is
     * automatically prepended to `idirectory` if it is not specified.
     *
     * @param ipathId The path ID that the paths are stored under (and will be loaded into)
     */
    void loadPath(const std::string &ipathId);

    /**
     * Attempts to remove a path without stopping execution. If that fails, disables the controller
     * and removes the path.
     *
     * @param ipathId The path ID that will be removed
     */
    void forceRemovePath(const std::string &ipathId);
protected:
    using TrajectoryPtr = std::unique_ptr<TrajectoryCandidate, void (*)(TrajectoryCandidate *)>;
    using SegmentPtr = std::unique_ptr<Segment, void (*)(void *)>;

    std::shared_ptr<OdomChassisController> chassis;
    std::shared_ptr<RamseteController> ramseteController;

    int timeout = 0;

    std::shared_ptr<Logger> logger;
    std::map<std::string, Trajectory> paths{};
    PathfinderLimits limits;
    std::shared_ptr<ChassisModel> model;
    Kinematics kinematics;
    ChassisScales scales;
    AbstractMotor::GearsetRatioPair pair;
    TimeUtil timeUtil;

    // This must be locked when accessing the current path
    CrossplatformMutex currentPathMutex;

    std::string currentPath{""};
    std::atomic_bool isRunning{false};
    std::atomic_int direction{1};
    std::atomic_bool shouldResetState{false};
    std::atomic_bool mirrored{false};
    std::atomic_bool disabled{false};
    std::atomic_bool dtorCalled{false};
    CrossplatformThread *task{nullptr};

    static void trampoline(void *context);
    void loop();

    /**
     * Follow the supplied trajectory. Must follow the disabled lifecycle.
     */
    virtual void executeSinglePath(const Trajectory &trajectory, 
                                   std::unique_ptr<AbstractRate> rate);

    std::string getPathErrorMessage(const std::vector<Waypoint> &points, const std::string &ipathId, 
                                    int length);

    QAngularSpeed convertLinearToRotational(QSpeed linear) const;

    /**
     * Joins and escapes a directory and file name
     *
     * @param directory The directory path, separated by forward slashes (/) and with or without a
     * trailing slash
     * @param filename The file name in the directory
     * @return the fully qualified and legal path name
     */
    static std::string makeFilePath(const std::string &directory, const std::string &filename);

    void internalStorePath(const std::string &ipathId,
                           SegmentPtr& trajectory, int length);
    void internalLoadPath(FILE *pathFile, const std::string &ipathId);

    /**
     * Reads the length of the trajectory in a thread-safe manner.
     *
     * @param trajectory The trajectory to read from.
     * @return The length of the trajectory.
     */
    int getTrajectoryLength(const Trajectory &trajectory);

    const Pose2d& getInitialPose(const Trajectory &trajectory);
};