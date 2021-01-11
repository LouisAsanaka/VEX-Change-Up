#include "main.h"
#include "autonomous.hpp"
#include "constants.hpp"
#include "robot.hpp"
#include "robot/conveyor.hpp"
#include "robot/drive.hpp"
#include "robot/intake.hpp"

#include "gui.hpp"

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
        // Controller master {ControllerId::master};
        // std::stringstream ss;
        // ss << "Angle: ";
        // ss << initialAngle.convert(degree);
        // ss << " deg";
        // std::cout << ss.str() << std::endl;
        // master.setText(0, 0, ss.str());
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

void rightSide1(bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(0_m, 0_m, 180_deg);
    }
    // Backup to release intake
    robot::drive->model->xArcade(0.0, -0.8, 0.0);
    pros::delay(100);
    robot::drive->model->xArcade(0.0, 0.6, 0.0);
    pros::delay(100);
    robot::drive->model->stop();
    pros::delay(100);

    // Strafe to face the corner goal
    robot::drive->controller->strafeToPose({0.04_m, 0.50_m, 225_deg}, 1500);
    pros::delay(60);

    // Intake the ball in front of the corner goal
    robot::drive->controller->driveForDistance(0.385_m, 500);
    robot::intake->spinIn(1.0);
    pros::delay(900);
    robot::intake->stop();
    // TODO(louis): ASYNC ACTION

    // Finish scoring the preload ball
    robot::drive->controller->driveForDistance(0.20_m, 500);
    robot::conveyor->startCountingBalls();
    robot::conveyor->moveBoth(1.0);
    robot::conveyor->waitUntilPassed(1);
    // TODO(louis): ASYNC ACTION
    //std::cout << "pop: " << Pose2d::fromOdomState(robot::drive->controller->getState()).toString() << std::endl;

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
    robot::intake->spinOut(1.0);
    robot::drive->controller->setMaxVoltage(0.85 * 12000);
    robot::drive->controller->strafeToPose({-0.93_m, 0.43_m, 180_deg}, 2500);
    pros::delay(500);
    robot::intake->stop();
    // TODO(louis): ASYNC ACTION
    // std::cout << "Final: " << robot::drive->controller->getState().str() << std::endl;
    // return;

    // std::stringstream ss;
    // ss << robot::drive->controller->getState().x.convert(meter);
    // ss << ", ";
    // ss << robot::drive->controller->getState().y.convert(meter);

    // Controller master {ControllerId::master};
    // master.setText(0, 0, ss.str());

    // Score the second ball by ramming into the goal & backing up
    robot::conveyor->startCountingBalls();
    robot::drive->model->xArcade(0.0, 0.7, 0.0);
    pros::delay(1000); // 1000 millis
    reset(-0.93_m, 0.12_m, 180_deg, true);
    backupFromGoal();
    //robot::drive->controller->driveForDistance(0.27_m);
    //pros::delay(150);
    robot::conveyor->moveBoth(1.0);
    robot::conveyor->waitUntilPassed(1);
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
    robot::drive->controller->strafeToPose({-2.05_m, 0.40_m, 135_deg}, 2000);
    robot::intake->spinOut(1.0);
    pros::delay(500);
    robot::intake->stop();
    // TODO(louis): ASYNC ACTION

    // Intake the ball in front of the third goal & score it
    robot::conveyor->startCountingBalls();
    robot::drive->controller->driveForDistance(0.35_m, 500);
    robot::intake->spinIn(1.0);
    // TODO(louis): ASYNC ACTION
    pros::delay(400);
    robot::intake->stop();

    robot::drive->model->xArcade(0.0, 0.7, 0.0);
    pros::delay(300);
    backupFromGoal();
    robot::conveyor->moveBoth(1.0);
    robot::conveyor->waitUntilPassed(1);

    // Backup and out-take
    robot::intake->spinOut(1.0);
    pros::delay(100);
    robot::drive->model->xArcade(0.0, -1.0, 0.0);
    pros::delay(800);
    robot::intake->stop();
    robot::drive->model->stop();
}
