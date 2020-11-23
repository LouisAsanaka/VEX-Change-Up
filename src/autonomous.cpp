#include "main.h"
#include "autonomous.hpp"
#include "constants.hpp"
#include "robot.hpp"
#include "robot/conveyor.hpp"
#include "robot/drive.hpp"
#include "robot/intake.hpp"

#include <exception>
#include <string>
#include <sstream>

void reset(okapi::QLength x, okapi::QLength y, 
           okapi::QAngle initialAngle, bool currentAngle) {
    if (currentAngle) {
        initialAngle = -(robot::drive::model->getSensorVals()[3] 
            / static_cast<double>(GYRO_RESOLUTION)) * degree;
        // Controller master {ControllerId::master};
        // std::stringstream ss;
        // ss << "Angle: ";
        // ss << initialAngle.convert(degree);
        // ss << " deg";
        // std::cout << ss.str() << std::endl;
        // master.setText(0, 0, ss.str());
    }
    robot::drive::resetEncoders();
    robot::drive::model->resetGyro(initialAngle);
    robot::drive::controller->setPose({x, y, initialAngle});
    robot::drive::controller->setMaxVoltage(1.0 * 12000);
}

void backout(int milliseconds) {
    robot::intake::spinOut(1.0);
    robot::drive::model->xArcade(0.0, -0.6, 0.0);
    pros::delay(milliseconds);
    robot::intake::stop();
    robot::drive::model->stop();
}

void backupFromGoal() {
    robot::drive::model->xArcade(0.0, -0.6, 0.0);
    pros::delay(100);
    robot::drive::model->stop();
}

void rightSide1(bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(0_m, 0_m, 180_deg);
    }
    // Backup to release intake
    robot::drive::model->xArcade(0.0, -0.6, 0.0);
    pros::delay(200);
    robot::drive::model->stop();
    pros::delay(100);

    // Strafe to face the corner goal
    robot::drive::controller->strafeToPoseAsync({0.04_m, 0.50_m, 225_deg});
    robot::drive::controller->waitUntilSettled(1500);
    pros::delay(60);

    // Intake the ball in front of the corner goal
    robot::drive::controller->driveForDistanceAsync(0.385_m);
    robot::intake::spinIn(1.0);
    pros::delay(900);
    robot::intake::stop();
    robot::drive::controller->waitUntilSettled(500);

    // Finish scoring the preload ball
    robot::drive::controller->driveForDistanceAsync(0.20_m);
    robot::conveyor::startCountingBalls();
    robot::conveyor::moveBoth(1.0);
    robot::conveyor::waitUntilPassed(1);
    robot::drive::controller->waitUntilSettled(500);
    //std::cout << "pop: " << Pose2d::fromOdomState(robot::drive::controller->getState()).toString() << std::endl;

    if (shouldBackOut) {
        backout(1000);
    }
}

void rightSide2(bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(0_m, 0_m, 180_deg);
    }
    rightSide1(false, false);
    pros::delay(50);

    // Backout to second goal
    robot::intake::spinOut(1.0);
    robot::drive::controller->setMaxVoltage(0.85 * 12000);
    robot::drive::controller->strafeToPoseAsync({-0.93_m, 0.43_m, 180_deg});
    pros::delay(500);
    robot::intake::stop();
    robot::drive::controller->waitUntilSettled(2500);
    // std::cout << "Final: " << robot::drive::controller->getState().str() << std::endl;
    // return;

    // std::stringstream ss;
    // ss << robot::drive::controller->getState().x.convert(meter);
    // ss << ", ";
    // ss << robot::drive::controller->getState().y.convert(meter);

    // Controller master {ControllerId::master};
    // master.setText(0, 0, ss.str());

    // Score the second ball by ramming into the goal & backing up
    robot::conveyor::startCountingBalls();
    robot::drive::model->xArcade(0.0, 0.7, 0.0);
    pros::delay(1000);
    reset(-0.93_m, 0.12_m, 180_deg, true);
    backupFromGoal();
    //robot::drive::controller->driveForDistanceAsync(0.27_m);
    //pros::delay(150);
    robot::conveyor::moveBoth(1.0);
    robot::conveyor::waitUntilPassed(1);
    robot::drive::controller->waitUntilSettled(100);
    if (shouldBackOut) {
        backout(1000);
    }
}

void rightSide3(bool shouldReset) {
    if (shouldReset) {
        reset(0_m, 0_m, 180_deg);
    }
    rightSide2(false, false);
    backout(700);
    
    // Move to third goal while out-taking
    robot::drive::controller->strafeToPoseAsync({-2.05_m, 0.40_m, 135_deg});
    robot::intake::spinOut(1.0);
    pros::delay(500);
    robot::intake::stop();
    robot::drive::controller->waitUntilSettled(2000);

    // Intake the ball in front of the third goal & score it
    robot::conveyor::startCountingBalls();
    robot::drive::controller->driveForDistanceAsync(0.35_m);
    robot::intake::spinIn(1.0);
    robot::drive::controller->waitUntilSettled(500);
    pros::delay(400);
    robot::intake::stop();

    robot::drive::model->xArcade(0.0, 0.7, 0.0);
    pros::delay(300);
    backupFromGoal();
    robot::conveyor::moveBoth(1.0);
    robot::conveyor::waitUntilPassed(1);

    // Backup and out-take
    robot::intake::spinOut(1.0);
    pros::delay(100);
    robot::drive::model->xArcade(0.0, -1.0, 0.0);
    pros::delay(800);
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
    rightSide3(true);

    std::stringstream ss;
    ss << "Time: ";
    ss << (pros::millis() - startTime) / 1000.0;
    ss << " s";

    std::cout << ss.str() << std::endl;

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