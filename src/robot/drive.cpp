#include "robot/drive.hpp"
#include "main.h"
#include "constants.hpp"
#include "libraidzero/api.hpp"
#include "okapi/impl/util/configurableTimeUtilFactory.hpp"
#include <memory>

namespace robot::drive {

	std::unique_ptr<MecanumOdomController> controller;
    // std::unique_ptr<MecanumController> mecanumController;
    //std::shared_ptr<AsyncRamsetePathController> pathFollower;
    //std::shared_ptr<AsyncAdvancedProfileController> profileFollower;
    std::shared_ptr<ThreeEncoderXDriveModel> model;

	void init() {
        IterativePosPIDController::Gains DISTANCE_GAINS {1.0, 0.0, 0.0};
        IterativePosPIDController::Gains ANGLE_GAINS {1.0, 0.0, 0.0};
        IterativePosPIDController::Gains TURN_GAINS {1.0, 0.0, 0.0};
        IterativePosPIDController::Gains STRAFE_DISTANCE_GAINS {1.0, 0.0, 0.0};
        IterativePosPIDController::Gains STRAFE_ANGLE_GAINS {1.0, 0.0, 0.0};

        AbstractMotor::gearset gearing {AbstractMotor::gearset::green};
        ChassisScales scales {
            {DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_TRACK}, 
		    okapi::imev5GreenTPR
        };
        ChassisScales odomScales {
            {DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_TRACK, TRACKING_BACK_WHEEL_DISTANCE, TRACKING_WHEEL_DIAMETER}, 
		    okapi::imev5GreenTPR
        };
        model = std::make_shared<ThreeEncoderXDriveModel>(
            std::make_shared<Motor>(1), 
            std::make_shared<Motor>(-2),
            std::make_shared<Motor>(-3),
            std::make_shared<Motor>(4),
            std::make_shared<ADIEncoder>('A', 'B'), 
            std::make_shared<ADIEncoder>('C', 'D', true),
            std::make_shared<ADIEncoder>('E', 'F'),
            toUnderlyingType(gearing), 12000
        );
        ConfigurableTimeUtilFactory closedLoopTimeFactory = ConfigurableTimeUtilFactory(
            50, 5, 100_ms);
        ConfigurableTimeUtilFactory strafingTimeFactory = ConfigurableTimeUtilFactory(
            0.02, 5, 100_ms);
        std::shared_ptr<Logger> controllerLogger {Logger::getDefaultLogger()};
        controller = std::make_unique<MecanumOdomController>(
            TimeUtilFactory::createDefault(),
            model,
            std::make_shared<ThreeEncoderOdometry>(
                TimeUtilFactory::createDefault(), model, odomScales, controllerLogger
            ),
            std::make_unique<IterativePosPIDController>(DISTANCE_GAINS,
                                                        closedLoopTimeFactory.create(),
                                                        std::make_unique<PassthroughFilter>(),
                                                        controllerLogger),
            std::make_unique<IterativePosPIDController>(ANGLE_GAINS,
                                                        closedLoopTimeFactory.create(),
                                                        std::make_unique<PassthroughFilter>(),
                                                        controllerLogger),
            std::make_unique<IterativePosPIDController>(TURN_GAINS,
                                                        closedLoopTimeFactory.create(),
                                                        std::make_unique<PassthroughFilter>(),
                                                        controllerLogger),
            std::make_unique<IterativePosPIDController>(STRAFE_DISTANCE_GAINS,
                                                        strafingTimeFactory.create(),
                                                        std::make_unique<PassthroughFilter>(),
                                                        controllerLogger),
            std::make_unique<IterativePosPIDController>(STRAFE_ANGLE_GAINS,
                                                        strafingTimeFactory.create(),
                                                        std::make_unique<PassthroughFilter>(),
                                                        controllerLogger),
            gearing, scales, 0.02_m, 1_deg, controllerLogger
        );
        // controller = std::static_pointer_cast<AdvancedOdomChassisController>(
        //     AdvancedChassisControllerBuilder()
        //         .withMotors(1, -2, -3, 4)
        //         .withDimensions(AbstractMotor::gearset::green, {
        //             {DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_TRACK}, 
		// 			okapi::imev5GreenTPR})
        //         .withClosedLoopControllerTimeUtil(50, 5, 100_ms)
        //         /*.withSensors(
        //             ADIEncoder{'A', 'B'}, // Left encoder in ADI ports A & B
        //             ADIEncoder{'C', 'D', true}, // Right encoder in ADI ports C & D (reversed)
        //             ADIEncoder{'E', 'F'} // Middle encoder
        //         )*/
        //         .withGains(
        //             {0.002, 0, 0.0001},
        //             {0.0013, 0, 0.0001},
        //             {0.001, 0, 0.0001}
        //         )
        //         .build()
        // );
        // mecanumController = std::make_unique<MecanumController>(
        //     TimeUtilFactory().create(), controller, 
        //     PIDController::Gains{1.0, 0.0, 0.0},
        //     PIDController::Gains{1.0, 0.0, 0.0}
        // );
        // mecanumController->flipDisable(true);
        /*pathFollower = AsyncRamsetePathControllerBuilder()
			.withLimits({DRIVE_MAX_VEL, DRIVE_MAX_ACCEL, DRIVE_MAX_JERK})
			.withOutput(controller)
			.buildRamsetePathController();
        pathFollower->flipDisable(true);*/

        /*profileFollower = AsyncAdvancedProfileControllerBuilder()
            .withConfig({DRIVE_MAX_VEL, DRIVE_MAX_ACCEL, 0.0})
            .withOutput(controller)
            .buildAdvancedProfileController();
        profileFollower->flipDisable(true);*/
		model->setBrakeMode(AbstractMotor::brakeMode::brake);
        model->setMaxVoltage(0.8 * 12000);
	}

    /*void generatePath(std::initializer_list<PathfinderPoint> iwaypoints,
                      const std::string &ipathId, bool storePath) {
        pathFollower->generatePath(iwaypoints, ipathId, storePath);
    }

    void followPath(const std::string& ipathId, bool resetState, bool ibackwards, bool imirrored) {
        pathFollower->setTarget(ipathId, resetState, ibackwards, imirrored);
        pathFollower->flipDisable(false);
    }

	OdomState getState() {
		return controller->getState();
	}*/

	void resetEncoders() {
		model->resetSensors();
	}

	void stop() {
		controller->stop();
	}
}
