#include "main.h"
#include "autonomous.hpp"
#include "constants.hpp"
#include "robot.hpp"
#include "libraidzero/planner/profilePlanner.hpp"
#include "robot/conveyor.hpp"
#include "robot/drive.hpp"
#include "robot/intake.hpp"

#include <exception>
#include <string>
#include <sstream>

void reset(okapi::QAngle initialAngle) {
    robot::drive::resetEncoders();
    robot::drive::model->resetGyro(initialAngle);
    robot::drive::controller->setPose({0_m, 0_m, initialAngle});
    robot::drive::controller->setMaxVoltage(1.0 * 12000);
}

void backout(int milliseconds) {
    robot::intake::spinOut(1.0);
    robot::drive::model->xArcade(0.0, -0.6, 0.0);
    pros::delay(milliseconds);
    robot::intake::stop();
    robot::drive::model->stop();
}

void rightSide1(bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(180_deg);
    }
    robot::drive::model->xArcade(0.0, -0.6, 0.0);
    pros::delay(200);
    robot::drive::model->stop();
    pros::delay(100);
    robot::drive::controller->strafeToPoseAsync({0.04_m, 0.50_m, 225_deg});
    robot::drive::controller->waitUntilSettled(3000);
    pros::delay(60);
    robot::drive::controller->driveForDistanceAsync(0.35_m);
    robot::intake::spinIn(1.0);
    pros::delay(900);
    robot::intake::stop();
    robot::drive::controller->waitUntilSettled(500);
    robot::drive::controller->driveForDistanceAsync(0.22_m);
    robot::conveyor::startCountingBalls();
    robot::conveyor::moveBoth(1.0);
    robot::conveyor::waitUntilPassed(1);
    robot::drive::controller->waitUntilSettled(500);
    std::cout << "pop: " << Pose2d::fromOdomState(robot::drive::controller->getState()).toString() << std::endl;

    if (shouldBackOut) {
        backout(1000);
    }
}

void rightSide2(bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(180_deg);
    }
    rightSide1(false, false);
    pros::delay(50);

    std::cout << "Ending1: " << Pose2d::fromOdomState(robot::drive::controller->getState()).toString() << std::endl;

    // Backout to second goal
    robot::intake::spinOut(1.0);
    robot::drive::controller->setMaxVoltage(0.85 * 12000);
    robot::drive::controller->strafeToPoseAsync({-0.95_m, 0.30_m, 180_deg});
    pros::delay(500);
    robot::intake::stop();
    robot::drive::controller->waitUntilSettled(3000);
    std::cout << "Final: " << robot::drive::controller->getState().str() << std::endl;
    return;

    std::stringstream ss;
    ss << robot::drive::controller->getState().x.convert(meter);
    ss << ", ";
    ss << robot::drive::controller->getState().y.convert(meter);

    Controller master {ControllerId::master};
    master.setText(0, 0, ss.str());

    robot::conveyor::startCountingBalls();
    robot::drive::controller->driveForDistanceAsync(0.25_m);
    pros::delay(150);
    robot::conveyor::moveBoth(1.0);
    robot::conveyor::waitUntilPassed(1);
    robot::drive::controller->waitUntilSettled(100);
    if (shouldBackOut) {
        backout(1000);
    }
}

void rightSide3(bool shouldReset) {
    if (shouldReset) {
        reset(180_deg);
    }
    rightSide2(false, false);
    backout(300);
    
    // Move to third stop
    robot::drive::controller->strafeToPoseAsync({-2.0_m, 0.40_m, 135_deg});
    robot::intake::spinOut(1.0);
    pros::delay(500);
    robot::intake::stop();
    robot::drive::controller->waitUntilSettled(2500);

    // Score third ball
    robot::conveyor::startCountingBalls();
    robot::drive::controller->driveForDistanceAsync(0.53_m);
    robot::intake::spinIn(1.0);
    robot::conveyor::moveBoth(1.0);
    robot::conveyor::waitUntilPassed(1);
    robot::intake::spinOut(1.0);
    robot::drive::controller->waitUntilSettled(100);
    robot::drive::model->xArcade(0.0, -0.6, 0.0);
    pros::delay(400);
    robot::intake::stop();
    robot::drive::model->stop();
}

void autonomous() {
    /*robot::drive::generatePath({
        {0_m, 0_m, 0_deg},
        {3.0_ft, 3.0_ft, 0_deg}
    }, "test");
    robot::drive::resetEncoders();

    robot::drive::followPath("test", true);*/
    auto startTime = pros::millis();
    // reset(180_deg);
    // robot::drive::controller->strafeToPoseAsync({0.04_m, 0.50_m, 225_deg});
    // robot::drive::controller->waitUntilSettled(5000);
    rightSide2(true, true);

    std::stringstream ss;
    ss << "Time: ";
    ss << (pros::millis() - startTime) / 1000.0;
    ss << " s";

    Controller master {ControllerId::master};
    master.setText(0, 0, ss.str());
    return;

    /*auto profile = planner::ProfilePlanner::generatePath(
        {
            {0_m, 0_m, 0_deg},
            {0.4_m, 0.4_m, -90_deg},
            {0.8_m, 0.0_m, 0_deg}
        }, planner::PlannerConfig{DRIVE_MAX_VEL, DRIVE_MAX_ACCEL, 0.0},
        true
    );
    Trajectory traj{Trajectory::profileToStates(profile)};

    robot::drive::controller->followTrajectoryAsync(traj);
    robot::drive::controller->waitUntilSettled();
    pros::delay(300);*/

    /*robot::drive::profileFollower->generatePath({
        {0_m, 0_m, 0_deg},
        {0.45_m, 0.45_m, 0_deg}
    }, "forward");
    robot::drive::profileFollower->generatePath({
        {0.45_m, 0.45_m, 180_deg},
        {0_m, 0_m, 180_deg}
    }, "backward");

    robot::drive::profileFollower->setTarget("forward");
    robot::drive::profileFollower->flipDisable(false);
    robot::drive::profileFollower->waitUntilSettled();
    robot::drive::profileFollower->setTarget("backward", true, true);
    robot::drive::profileFollower->flipDisable(false);
    robot::drive::profileFollower->waitUntilSettled();*/
}