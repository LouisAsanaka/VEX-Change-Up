#include "robot/drive.hpp"
#include "libraidzero/filter/slewRateLimiter.hpp"
#include "main.h"
#include "libraidzero/api.hpp"
#include "constants.hpp"
#include "okapi/api/util/mathUtil.hpp"
#include "okapi/impl/util/configurableTimeUtilFactory.hpp"

namespace robot {

Drive::Drive() {
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
    auto gyro = std::make_shared<BetterIMU>(19, IMUAxes::z);
    model = std::make_shared<ThreeEncoderImuXDriveModel>(
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
        std::make_shared<ThreeEncoderImuOdometry>(
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

    model->setBrakeMode(AbstractMotor::brakeMode::brake);
    model->setMaxVoltage(DRIVE_SPEED * 12000);
    model->setMaxVelocity(
        DRIVE_SPEED * toUnderlyingType(gearing.internalGearset)
    );

    pros::delay(1000);
    gyro->calibrate();
}

void Drive::fieldOrientedControl(double irightSpeed, double iforwardSpeed, 
    double iyaw, double ithreshold) 
{
    auto angle = -model->getHeading() * degree;
    //std::cout << "Angle: " << angle.convert(degree) << std::endl;
    Translation2d input {irightSpeed * meter, iforwardSpeed * meter};
    input = input.rotateBy(Rotation2d{-angle});
    model->xArcade(input.x().convert(meter), input.y().convert(meter), 
        iyaw, ithreshold);
}

void Drive::resetEncoders() {
    model->resetSensors();
    pros::delay(50);
}

void Drive::stop() {
    controller->stop();
}
} // namespace robot
