#include "main.h"
#include "autonomous.hpp"
#include "constants.hpp"
#include "pros/rtos.hpp"
#include "robot.hpp"
#include "robot/conveyor.hpp"
#include "robot/drive.hpp"
#include "robot/intake.hpp"

#include "gui.hpp"

using RollerPosition = robot::Conveyor::RollerPosition;
using BallPosition = robot::Conveyor::BallPosition;

void autonomous() {
    #if defined(RUN_WITHOUT_ROBOT) && RUN_WITHOUT_ROBOT 
	return;
	#endif

    auto startTime = pros::millis();

    const std::string& selectedAuton = GUI::getInstance().selectedAuton;

    if (selectedAuton == "Right 1") {
        rightSide1(true, true);
    } else if (selectedAuton == "Right 2") {
        rightSide2(true, true);
    } else if (selectedAuton == "Right 3") {
        rightSide3(true);
    } else {
        reset(0_m, 0_m, 0_deg, false);
        rightSide3(true);
        // robot::intake->spinIn(1.0);
        // robot::conveyor->moveBoth(1.0);
        // //robot::conveyor->startCountingPassed(BallPosition::Top);
        // robot::conveyor->waitUntilEmpty(BallPosition::Top);
        // robot::conveyor->stopAll();
        // robot::intake->stop();
        // std::shared_ptr<PurePursuitPath> path = std::make_shared<PurePursuitPath>(
        //     std::vector<Pose2d>{
        //         {0.6_m, 0_m, 0_deg},
        //         {1_m, 1_m, 0_deg}
        //     }, 0.02_m,
        //     PurePursuitPath::Constraints{0.6, 1.0, 2.0}
        // );
        // double b = 0.98;
        // double a = 1 - b;
        // double tol = 0.001;
        // path->smoothen(a, b, tol);
        // path->fillPointInformation();

        // for (const auto& point : path->points) {
        //     std::cout << point.pose.toString() << " => " << point.distanceFromStart << "m, " << point.curvature << " curvature, " << point.targetVelocity << " m/s" << std::endl;
        // }
        // robot::drive->controller->followPath(path, 0.3_m, 0.01_m, {0.01, 1.0, 15.0});
        // std::cout << Pose2d::fromOdomState(robot::drive->controller->getState()).toString() << std::endl;
    }

    std::stringstream ss;
    ss << "Time: ";
    ss << (pros::millis() - startTime) / 1000.0;
    ss << " s";

    std::cout << ss.str() << std::endl;

    Controller master {ControllerId::master};
    master.setText(0, 0, ss.str());
}

void reset(okapi::QLength x, okapi::QLength y, 
           okapi::QAngle initialAngle, bool currentAngle) {
    if (currentAngle) {
        initialAngle = -robot::drive->model->getHeading() * degree;
    }
    robot::drive->resetEncoders();
    robot::drive->model->resetImu(initialAngle);
    robot::drive->controller->setPose({x, y, initialAngle});
    robot::drive->controller->setMaxVoltage(1.0 * 12000);
}

void backout(int milliseconds) {
    robot::intake->spinOut(1.0);
    robot::drive->model->xArcade(0.0, -0.6, 0.0);
    pros::delay(milliseconds);
    robot::intake->stop();
    robot::drive->model->stop();
}

void backupFromGoal() {
    robot::drive->model->xArcade(0.0, -0.6, 0.0);
    pros::delay(100);
    robot::drive->model->stop();
}

void releaseComponents() {
    // Backup to release intake
    //robot::drive->model->xArcade(0.0, -1.0, 0.0);
    //pros::delay(400);
    robot::conveyor->moveUp(0.8, RollerPosition::Top);
    //robot::drive->model->xArcade(0.0, 1.0, 0.0);
    pros::delay(100);
    robot::conveyor->stop(RollerPosition::Top);
    //robot::drive->model->stop();
}

void rightSide1(bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(0_m, 0_m, 0_deg);
    }
    releaseComponents();

    robot::intake->spinIn(1.0);

    // Strafe to face the corner goal
    robot::drive->controller->setMaxVoltage(0.5 * 12000);
    robot::drive->controller->strafeToPose({0.0_m, 0.15_m, 0_deg}, 800);
    pros::delay(100);
    robot::intake->stop();

    robot::drive->controller->setMaxVoltage(1.0 * 12000);
    robot::drive->controller->strafeToPose({-0.1_m, 0.3_m, -48_deg}, 1000);
    robot::drive->model->xArcade(0.0, 1.0, 0.0);
    pros::delay(200);
    robot::drive->model->stop();  

    robot::conveyor->moveBoth(1.0);
    robot::conveyor->startCountingPassed(BallPosition::Top);
    robot::conveyor->waitUntilPassed(1, 1200);
    //pros::delay(50);
    robot::conveyor->stopAll();

    if (shouldBackOut) {
        backout(1000);
    }
}

void rightSide2(bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(0_m, 0_m, 0_deg);
    }
    rightSide1(false, false);

    // Backout to second goal
    robot::intake->spinOut(1.0);
    backout(700);
    robot::intake->stop();

    robot::drive->controller->strafeToPose({-0.35_m, -1.05_m, -90_deg}, 3000);
    
    robot::drive->controller->driveForDistance(0.18_m, 700);
    robot::conveyor->moveBoth(1.0);
    robot::conveyor->startCountingPassed(BallPosition::Top);
    robot::conveyor->waitUntilPassed(1, 2000);
    robot::conveyor->stopAll();

    if (shouldBackOut) {
        backout(1000);
    }
}

void rightSide3(bool shouldReset) {
    if (shouldReset) {
        reset(0_m, 0_m, 0_deg);
    }
    rightSide2(false, false);
    backout(300);
    
    // Move to third goal while out-taking
    robot::intake->spinOut(1.0);
    robot::drive->controller->strafeToPose({-0.4_m, -2.0_m, -135_deg}, 2000);
    robot::intake->stop();

    // Intake the ball in front of the third goal & score it
    robot::intake->spinIn(1.0);
    robot::drive->controller->strafeToPose({-0.2_m, -2.4_m, -135_deg}, 1000);
    //robot::drive->controller->driveForDistance(0.42_m, 1000);
    pros::delay(200);
    robot::intake->stop();

    robot::drive->controller->driveForDistance(0.3_m, 1000);

    robot::conveyor->moveBoth(1.0);
    robot::conveyor->startCountingPassed(BallPosition::Top);
    robot::conveyor->waitUntilPassed(1, 2000);
    robot::conveyor->stopAll();

    // Backup and out-take
    robot::intake->spinOut(1.0);
    robot::drive->model->xArcade(0.0, -1.0, 0.0);
    pros::delay(800);
    robot::intake->stop();
    robot::drive->model->stop();
    return;
}

void goingMid() {
    reset(0_m, 0_m, 0_deg);
    rightSide1(true, false);
    // Backout to second goal
    robot::intake->spinIn(1.0);
    robot::drive->model->xArcade(0.0, -0.8, 0.0);
    //robot::conveyor->startIndexing(robot::Conveyor::ControlMode::StoreBall);
    robot::conveyor->moveBoth(1.0);
    //robot::conveyor->waitUntilStored(RollerPosition::Top);
    robot::conveyor->stopAll();
    robot::intake->stop();
    robot::drive->model->stop();
    robot::drive->controller->strafeToPose({-0.73_m, -1.31_m, -225_deg}, 4000);
    robot::drive->model->xArcade(0.0, 0.4, 0.0);
    pros::delay(100);
    robot::intake->spinOut(1.0);
    //robot::conveyor->startIndexing(robot::Conveyor::ControlMode::WaitUntilEmpty);
    robot::conveyor->moveBoth(-1.0);
    //robot::conveyor->waitUntilEmpty(RollerPosition::Top);
    pros::delay(400);
    robot::conveyor->stopAll();
    pros::delay(100);
    robot::intake->stop();
    robot::drive->controller->strafeToPose({-0.95_m, -0.43_m, 0_deg}, 4000);
    // Score the second ball by ramming into the goal & backing up
    robot::drive->model->xArcade(0.0, 0.7, 0.0);
    pros::delay(500); // 1000 millis
    backupFromGoal();
    robot::conveyor->moveBoth(1.0);
    pros::delay(1100);
    robot::conveyor->stopAll();
    pros::delay(100);
    robot::drive->model->xArcade(0.0, -1.0, 0.0);
    pros::delay(1000);
    robot::drive->model->stop();
}
