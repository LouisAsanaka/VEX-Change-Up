#pragma once

#include "main.h"
#include "controller/asyncRamsetePathController.hpp"

namespace robot::drive {

	extern std::shared_ptr<OdomChassisController> controller;
    extern std::shared_ptr<AsyncRamsetePathController> pathFollower;
	extern std::shared_ptr<ChassisModel> model;

	void init();

    void generatePath(std::initializer_list<PathfinderPoint> iwaypoints,
                      const std::string &ipathId, bool storePath = false);

    void followPath(const std::string& ipathId, bool resetState = false, bool ibackwards = false, 
                    bool imirrored = false);

	/**
	 * Returns the current odometry state
	 */
	const OdomState getState();

    /**
     * Resets all the encoders on the drivetrain 
     */
	void resetEncoders();

    /**
     * Stops all the motors on the drivetrain
     */
	void stop();
}
