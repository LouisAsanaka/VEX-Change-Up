#pragma once

#include "main.h"
#include "libraidzero/api.hpp"

namespace robot {

class Drive {
public:
    std::unique_ptr<XOdomController> controller;
	std::shared_ptr<ThreeEncoderImuXDriveModel> model;

    Drive();

    /**
     * Does field-oriented control for the x-drive.
     */
    void fieldOrientedControl(double irightSpeed, double iforwardSpeed, 
        double iyaw, double ithreshold = 0.05);

    /**
     * Resets all the encoders on the drivetrain 
     */
	void resetEncoders();

    /**
     * Stops all the motors on the drivetrain
     */
	void stop() const;
};

}
