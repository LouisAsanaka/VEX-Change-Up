#include "robot/drive.hpp"
#include "main.h"
#include "constants.hpp"
#include "libraidzero/controller/asyncRamsetePathController.hpp"
#include "libraidzero/builder/asyncRamsetePathControllerBuilder.hpp"

namespace robot::drive {

	std::shared_ptr<OdomChassisController> controller;
    std::shared_ptr<AsyncRamsetePathController> pathFollower;
    std::shared_ptr<ChassisModel> model;

	void init() {
        controller = std::static_pointer_cast<OdomChassisController>(
            ChassisControllerBuilder()
                .withMotors({9, 10}, {-1, -2})
                .withDimensions(AbstractMotor::gearset::green, {
                    {DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_TRACK}, 
					okapi::imev5GreenTPR})
                .withClosedLoopControllerTimeUtil(50, 5, 100_ms)
                .withSensors(
                    ADIEncoder{'A', 'B'}, // Left encoder in ADI ports A & B
                    ADIEncoder{'C', 'D', true}, // Right encoder in ADI ports C & D (reversed)
                    ADIEncoder{'E', 'F'} // Middle encoder
                )
                .withGains(
                    {0.0023, 0, 0.0001},
                    {0.002, 0, 0.0001},
                    {0.001, 0, 0.0001}
                )
                .withOdometry({{
                    TRACKING_WHEEL_DIAMETER, TRACKING_WHEEL_TRACK,
                    TRACKING_BACK_WHEEL_DISTANCE, TRACKING_WHEEL_DIAMETER}, 
                    quadEncoderTPR},
                    StateMode::FRAME_TRANSFORMATION, 10_mm, 2_deg)
                .buildOdometry()
        );
        pathFollower = AsyncRamsetePathControllerBuilder()
			.withLimits({DRIVE_MAX_VEL, DRIVE_MAX_ACCEL, DRIVE_MAX_JERK})
			.withOutput(controller)
			.buildRamsetePathController();
		model = controller->getModel();
		model->setBrakeMode(AbstractMotor::brakeMode::brake);
	}

    void generatePath(std::initializer_list<PathfinderPoint> iwaypoints,
                      const std::string &ipathId, bool storePath) {
        pathFollower->generatePath(iwaypoints, ipathId, storePath);
    }

    void followPath(const std::string& ipathId, bool resetState, bool ibackwards, bool imirrored) {
        pathFollower->setTarget(ipathId, resetState, ibackwards, imirrored);
    }

	const OdomState getState() {
		return controller->getState();
	}

	void resetEncoders() {
		model->resetSensors();
	}

	void stop() {
		controller->stop();
	}
}
