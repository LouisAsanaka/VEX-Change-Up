#include "robot/drive.hpp"
#include "main.h"
#include "constants.hpp"
#include "libraidzero/api.hpp"
#include "okapi/impl/util/configurableTimeUtilFactory.hpp"

namespace robot::drive {

	std::unique_ptr<XOdomController> controller;
    //std::shared_ptr<AsyncRamsetePathController> pathFollower;
    std::shared_ptr<AsyncAdvancedProfileController> profileFollower;
    std::shared_ptr<XDriveModel> model;

	void init() {
        IterativePosPIDController::Gains DISTANCE_GAINS {0.002, 0.0, 0.0001};
        IterativePosPIDController::Gains ANGLE_GAINS {0.002, 0.0, 0.0};
        IterativePosPIDController::Gains TURN_GAINS {0.003, 0.0, 0.0001};
        IterativePosPIDController::Gains STRAFE_DISTANCE_GAINS {1.0, 0.0, 0.0};
        IterativePosPIDController::Gains STRAFE_ANGLE_GAINS {1.0, 0.0, 0.0};

        AbstractMotor::GearsetRatioPair gearing {AbstractMotor::gearset::green, 1.0};
        ChassisScales scales {
            {DRIVE_WHEEL_DIAMETER, DRIVE_WHEEL_TRACK, 
            TRACKING_BACK_WHEEL_DISTANCE, TRACKING_WHEEL_DIAMETER}, 
		    okapi::imev5GreenTPR
        };
        auto leftLeader = std::make_shared<Motor>(1);
        auto rightLeader = std::make_shared<Motor>(-2);
        model = std::make_shared<ThreeEncoderXDriveModel>(
            leftLeader, 
            rightLeader,
            std::make_shared<Motor>(-3),
            std::make_shared<Motor>(4),
            std::make_shared<ADIEncoder>('A', 'B'), 
            std::make_shared<ADIEncoder>('C', 'D', true),
            std::make_shared<ADIEncoder>('E', 'F'),
            toUnderlyingType(gearing.internalGearset), 12000
        );
        ConfigurableTimeUtilFactory closedLoopTimeFactory = ConfigurableTimeUtilFactory(
            50, 5, 100_ms);
        ConfigurableTimeUtilFactory strafingTimeFactory = ConfigurableTimeUtilFactory(
            0.02, 5, 100_ms);
        std::shared_ptr<Logger> controllerLogger {Logger::getDefaultLogger()};
        controller = std::make_unique<XOdomController>(
            TimeUtilFactory::createDefault(),
            model,
            std::make_shared<ThreeEncoderOdometry>(
                TimeUtilFactory::createDefault(), model, scales, controllerLogger
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
        controller->startOdomThread();
        if (NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
            controller->getOdomThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
        }
        controller->startThread();
        if (NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
            controller->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
        }

        profileFollower = std::make_shared<AsyncAdvancedProfileController>(
            TimeUtilFactory::createDefault(),
            planner::PlannerConfig{DRIVE_MAX_VEL, DRIVE_MAX_ACCEL, 0.0},
            model, scales, gearing, controllerLogger
        );
        profileFollower->startThread();
        if (NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
            profileFollower->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
        }
        profileFollower->flipDisable(true);

        model->setBrakeMode(AbstractMotor::brakeMode::brake);
        model->setMaxVoltage(1.0 * 12000);

        /*pathFollower = AsyncRamsetePathControllerBuilder()
			.withLimits({DRIVE_MAX_VEL, DRIVE_MAX_ACCEL, DRIVE_MAX_JERK})
			.withOutput(controller)
			.buildRamsetePathController();
        pathFollower->flipDisable(true);*/
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
