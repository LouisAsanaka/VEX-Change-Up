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
        runSide1(StartingSide::Right, true, true);
    } else if (selectedAuton == "Right 2") {
        runSide2(StartingSide::Right, true, true);
    } else if (selectedAuton == "Right 3") {
        runSide3(StartingSide::Right, true, true);
    } else {
        reset(0_m, 0_m, 0_deg, false);
        runSideCenter(StartingSide::Left, true, true);
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
    // Release hood by moving the top conveyor
    robot::conveyor->moveBoth(-1.0);
    pros::delay(100);
    robot::conveyor->stopAll();
    pros::delay(200);
}

void scoreOneBall() {
    robot::conveyor->moveBoth(1.0);
    robot::conveyor->startCountingPassed(BallPosition::Top);
    robot::conveyor->waitUntilPassed(1, 1200);
    robot::conveyor->stop(RollerPosition::Bottom);
    pros::delay(50);
    robot::conveyor->stop(RollerPosition::Top);
}

void runSide1(StartingSide side, bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(0_m, 0_m, 0_deg);
    }
    releaseComponents();

    // Intake the corner ball
    robot::intake->spinIn(1.0);
    robot::drive->controller->setMaxVoltage(0.5 * 12000);
    robot::drive->controller->strafeToPose({0.0_m, 0.15_m, 0_deg}, 800);
    pros::delay(200);
    robot::intake->stop();

    // Drive and angle towards the goal
    robot::drive->controller->setMaxVoltage(1.0 * 12000);
    if (side == StartingSide::Right) {
        robot::drive->controller->strafeToPose({-0.1_m, 0.3_m, -48_deg}, 1000);
    } else {
        robot::drive->controller->strafeToPose({0.1_m, 0.3_m, 48_deg}, 1000);
    }
    
    robot::drive->model->xArcade(0.0, 1.0, 0.0);
    pros::delay(300);
    robot::drive->model->stop();  

    // Score one ball
    scoreOneBall();
    pros::delay(50);

    if (shouldBackOut) {
        backout(1000);
    }
}

void runSide2(StartingSide side, bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(0_m, 0_m, 0_deg);
    }
    runSide1(side, false, false);

    // Backout to second goal
    robot::conveyor->moveDown(1.0, RollerPosition::Top);
    robot::intake->spinOut(1.0);
    backout(700);
    robot::intake->stop();
    robot::conveyor->stop(RollerPosition::Top);

    // Strafing to the home row middle goal
    if (side == StartingSide::Right) {
        robot::drive->controller->strafeToPose({-0.35_m, -1.05_m, -90_deg}, 3000);
    } else {
        robot::drive->controller->strafeToPose({0.35_m, -1.05_m, 90_deg}, 3000);
    }
    // Drive towards the goal and score one ball
    robot::drive->controller->driveForDistance(0.21_m, 800);
    scoreOneBall();
    pros::delay(50);

    if (shouldBackOut) {
        backout(1000);
    }
}

void runSide3(StartingSide side, bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(0_m, 0_m, 0_deg);
    }
    runSide2(side, false, false);
    backout(300);
    
    // Move to third goal while out-taking
    robot::intake->spinOut(1.0);
    if (side == StartingSide::Right) {
        robot::drive->controller->strafeToPose({-0.35_m, -2.16_m, -135_deg}, 2500);
    } else {
        robot::drive->controller->strafeToPose({0.35_m, -2.16_m, 135_deg}, 2500);
    }
    robot::intake->stop();

    // Intake the ball in front of the third goal
    robot::intake->spinIn(1.0);
    robot::drive->controller->driveForDistance(0.40_m, 1000);
    pros::delay(200);
    robot::intake->stop();

    // Drive towards the goal and score one ball
    robot::drive->controller->driveForDistance(0.05_m, 800);
    scoreOneBall();
    pros::delay(50);

    if (shouldBackOut) {
        // Backup and out-take
        robot::intake->spinOut(1.0);
        backout(1000);
        robot::intake->stop();
    }
}

void runSideCenter(StartingSide side, bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(0_m, 0_m, 0_deg);
    }
    runSide1(side, false, false);
    if (side == StartingSide::Right) {
        robot::drive->controller->driveForDistance(0.05_m, 500);
    
        // Intake the bottom ball
        robot::intake->spinIn(1.0);

        robot::conveyor->moveUp(1.0, RollerPosition::Bottom);
        robot::conveyor->waitUntilStored(BallPosition::Top, 1000);
        robot::conveyor->waitUntilStored(BallPosition::Middle, 2000);
        robot::conveyor->stop(RollerPosition::Bottom);
        pros::delay(300);
        robot::intake->stop();

        robot::conveyor->moveDown(0.5, RollerPosition::Top);
        robot::conveyor->moveDown(1.0, RollerPosition::Bottom);
        
        robot::intake->spinOut(1.0);
        pros::delay(200);
        robot::conveyor->stopAll();
        backout(600);
        robot::intake->stop();

        robot::drive->controller->strafeToPose({-1.13_m, -0.87_m, 135_deg}, 3000);

        robot::drive->controller->driveForDistance(0.10_m, 800);
        scoreOneBall();
        pros::delay(50);

        robot::drive->controller->strafeToPose({-0.18_m, -1.06_m, -90_deg}, 3000);
        robot::drive->controller->driveForDistance(0.04_m, 800);
        scoreOneBall();
        backout(500);
    } else {
        robot::conveyor->moveDown(0.5, RollerPosition::Top);
        robot::conveyor->moveDown(1.0, RollerPosition::Bottom);
        
        robot::intake->spinOut(1.0);
        pros::delay(200);
        robot::conveyor->stopAll();
        backout(600);
        robot::intake->stop();

        // Intake the centerline ball
        robot::intake->spinIn(1.0);
        robot::conveyor->moveUp(1.0, RollerPosition::Bottom);

        robot::drive->controller->strafeToPose({1.28_m, -0.22_m, -90_deg}, 3000);
        robot::conveyor->waitUntilStored(BallPosition::Top, 1000);
        robot::conveyor->waitUntilStored(BallPosition::Middle, 1000);
        robot::conveyor->stop(RollerPosition::Bottom);
        pros::delay(300);
        robot::intake->stop();

        // Score center
        robot::drive->controller->strafeToPose({1.13_m, -0.87_m, -135_deg}, 3000);

        robot::drive->controller->driveForDistance(0.10_m, 800);
        scoreOneBall();
        pros::delay(50);

        robot::drive->controller->strafeToPose({0.18_m, -1.06_m, 90_deg}, 3000);
        robot::drive->controller->driveForDistance(0.04_m, 800);
        scoreOneBall();
        backout(500);
    }
    
    
}
