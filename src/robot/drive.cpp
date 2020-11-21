#include "robot/drive.hpp"
#include "libraidzero/filter/slewRateLimiter.hpp"
#include "main.h"
#include "libraidzero/api.hpp"
#include "constants.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/util/configurableTimeUtilFactory.hpp"

namespace robot::drive {

	std::unique_ptr<XOdomController> controller;
    //std::shared_ptr<AsyncRamsetePathController> pathFollower;
    std::shared_ptr<AsyncAdvancedProfileController> profileFollower;
    std::shared_ptr<ThreeEncoderGyroXDriveModel> model;

	void init() {
        IterativePosPIDController::Gains DISTANCE_GAINS {
            DISTANCE_KP, DISTANCE_KI, DISTANCE_KD
        };
        IterativePosPIDController::Gains ANGLE_GAINS {
            ANGLE_KP, ANGLE_KI, ANGLE_KD
        };
        IterativePosPIDController::Gains TURN_GAINS {
            TURN_KP, TURN_KI, TURN_KD
        };
        IterativePosPIDController::Gains STRAFE_DISTANCE_GAINS {
            STRAFE_DISTANCE_KP, STRAFE_DISTANCE_KI, STRAFE_DISTANCE_KD
        };
        IterativePosPIDController::Gains STRAFE_ANGLE_GAINS {
            STRAFE_ANGLE_KP, STRAFE_ANGLE_KI, STRAFE_ANGLE_KD
        };

        AbstractMotor::GearsetRatioPair gearing {AbstractMotor::gearset::green, 1.0};
        ChassisScales odomScales {
            {TRACKING_WHEEL_DIAMETER, WHEEL_TRACK, 
            TRACKING_BACK_WHEEL_DISTANCE, TRACKING_WHEEL_DIAMETER}, 
		    okapi::quadEncoderTPR
        };
        auto leftLeader = std::make_shared<Motor>(1);
        auto rightLeader = std::make_shared<Motor>(-2);
        auto leftEnc = std::make_shared<ADIEncoder>('C', 'D', true);
        auto rightEnc = std::make_shared<ADIEncoder>('G', 'H', false);
        auto midEnc = std::make_shared<ADIEncoder>('E', 'F', true);
        auto gyro = std::make_shared<pros::Imu>(19);
        model = std::make_shared<ThreeEncoderGyroXDriveModel>(
            leftLeader,
            rightLeader,
            std::make_shared<Motor>(-3),
            std::make_shared<Motor>(4),
            leftEnc, rightEnc, midEnc,
            gyro,
            toUnderlyingType(gearing.internalGearset), 12000
        );
        ConfigurableTimeUtilFactory closedLoopTimeFactory = ConfigurableTimeUtilFactory(
            DRIVE_ENCODER_TARGET_ERROR, DRIVE_ENCODER_TARGET_DERIV, DRIVE_ENCODER_TARGET_TIME
        );
        ConfigurableTimeUtilFactory strafingDistTimeFactory = ConfigurableTimeUtilFactory(
            STRAFING_DIST_TARGET_ERROR, STRAFING_DIST_TARGET_DERIV, STRAFING_DIST_TARGET_TIME
        );
        ConfigurableTimeUtilFactory strafingAngleTimeFactory = ConfigurableTimeUtilFactory(
            STRAFING_ANGLE_TARGET_ERROR, STRAFING_ANGLE_TARGET_DERIV, STRAFING_ANGLE_TARGET_TIME
        );
        std::shared_ptr<Logger> controllerLogger {Logger::getDefaultLogger()};

        controller = std::make_unique<XOdomController>(
            TimeUtilFactory::createDefault(),
            model,
            std::make_shared<ThreeEncoderGyroOdometry>(
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
                                                        strafingDistTimeFactory.create(),
                                                        std::make_unique<PassthroughFilter>(),
                                                        controllerLogger),
            std::make_unique<IterativePosPIDController>(STRAFE_ANGLE_GAINS,
                                                        strafingAngleTimeFactory.create(),
                                                        std::make_unique<PassthroughFilter>(),
                                                        controllerLogger),
            std::make_unique<SlewRateLimiter>(STRAFE_SLEW_RATE),
            gearing, odomScales, DISTANCE_BEFORE_MOVE, ANGLE_BEFORE_TURN, controllerLogger
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
            model, odomScales, gearing, controllerLogger
        );
        profileFollower->startThread();
        if (NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
            profileFollower->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
        }
        profileFollower->flipDisable(true);

        model->setBrakeMode(AbstractMotor::brakeMode::brake);
        model->setMaxVoltage(DRIVE_SPEED * 12000);
        model->setMaxVelocity(
            DRIVE_SPEED * toUnderlyingType(gearing.internalGearset)
        );

        pros::delay(1000);
        gyro->reset();
        while (gyro->is_calibrating()) { // Stop for about 2 seconds
            pros::delay(500);
        }

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

    void fieldOrientedControl(double irightSpeed, double iforwardSpeed, 
        double iyaw, double ithreshold) 
    {
        auto angle = -model->getSensorVals()[3] / GYRO_RESOLUTION * degree;
        //std::cout << "Angle: " << angle.convert(degree) << std::endl;
        Translation2d input {irightSpeed * meter, iforwardSpeed * meter};
        input = input.rotateBy(Rotation2d{-angle});
        model->xArcade(input.x().convert(meter), input.y().convert(meter), 
            iyaw, ithreshold);
    }

	void resetEncoders() {
		model->resetSensors();
        pros::delay(50);
	}

	void stop() {
		controller->stop();
	}
}
