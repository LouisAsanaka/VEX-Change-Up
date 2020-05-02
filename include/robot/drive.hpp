#pragma once

#include "main.h"
#include "libraidzero/api.hpp"

namespace robot::drive {

	extern std::unique_ptr<MecanumOdomController> controller;
    // extern std::unique_ptr<MecanumController> mecanumController;
    //extern std::shared_ptr<AsyncRamsetePathController> pathFollower;
    //extern std::shared_ptr<AsyncAdvancedProfileController> profileFollower;
	extern std::shared_ptr<ThreeEncoderXDriveModel> model;

	void init();

    /*
    void generatePath(std::initializer_list<PathfinderPoint> iwaypoints,
                      const std::string &ipathId, bool storePath = false);

    void followPath(const std::string& ipathId, bool resetState = false, bool ibackwards = false, 
                    bool imirrored = false);

	
	 * Returns the current odometry state
	OdomState getState();*/

    /**
     * Resets all the encoders on the drivetrain 
     */
	void resetEncoders();

    /**
     * Stops all the motors on the drivetrain
     */
	void stop();
}
