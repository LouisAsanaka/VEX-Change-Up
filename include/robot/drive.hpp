#pragma once

#include "main.h"

namespace robot::drive {
	extern std::shared_ptr<OdomChassisController> controller;
	extern std::shared_ptr<ChassisModel> model;

	void init();

	/**
	 * Returns the current odometry state
	 */
	const OdomState& getState();

    /**
     * Resets all the encoders on the drivetrain 
     */
	void resetEncoders();

    /**
     * Stops all the motors on the drivetrain
     */
	void stop();
}
