#include "main.h"
#include "autonomous.hpp"
#include "constants.hpp"
#include "pros/rtos.hpp"
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
        rightSide3(true);
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
    robot::drive->model->xArcade(0.0, -1.0, 0.0);
    pros::delay(400);
    robot::conveyor->moveDown(0.6, robot::Conveyor::Position::Top);
    robot::drive->model->xArcade(0.0, 1.0, 0.0);
    pros::delay(100);
    robot::conveyor->stop(robot::Conveyor::Position::Top);
    robot::drive->model->stop();
}

void rightSide1(bool shouldReset, bool shouldBackOut) {
    if (shouldReset) {
        reset(0_m, 0_m, 0_deg);
    }
    releaseComponents();

    // Strafe to face the corner goal
    robot::drive->controller->strafeToPose({-0.04_m, -0.55_m, 45_deg}, 1400);

    robot::intake->spinIn(1.0);
    // Intake the ball in front of the corner goal
    robot::drive->controller->driveForDistance(0.7_m, 500);
    pros::delay(700);
    robot::intake->stop();
    // TODO(louis): ASYNC ACTION

    // Finish scoring the preload ball
    robot::conveyor->startIndexing(robot::Conveyor::ControlMode::StoreBall);
    robot::conveyor->moveUp(1.0, robot::Conveyor::Position::Bottom);
    int time1 = pros::millis();
    robot::conveyor->stopAll();
    if (pros::millis() - time1 < 600) {
        pros::delay(600 - (pros::millis() - time1));
    }
    robot::drive->model->stop();
    // backupFromGoal();
    // pros::delay(10);
    // robot::drive->model->stop(); // BIG BUG DON'T DELETE
    robot::conveyor->moveBoth(1.0);
    pros::delay(50);
    robot::conveyor->stop(robot::Conveyor::Position::Bottom);
    pros::delay(500);
    robot::conveyor->stopAll();
    pros::delay(50);
    // TODO(louis): ASYNC ACTION
    //std::cout << "pop: " << Pose2d::fromOdomState(robot::drive->controller->getState()).toString() << std::endl;

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
    backout(500);
    robot::intake->stop();
    //robot::drive->controller->setMaxVoltage(0.85 * 12000);

    // std::stringstream ss;
    // ss << robot::drive->controller->getState().x.convert(meter);
    // ss << ", ";
    // ss << robot::drive->controller->getState().y.convert(meter);

    // Controller master {ControllerId::master};
    // master.setText(0, 0, ss.str());

    robot::drive->controller->strafeToPose({0.91_m, -0.43_m, 0_deg}, 3000);
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
    robot::drive->model->xArcade(0.0, 0.7, 0.0);
    pros::delay(500); // 1000 millis
    reset(0.93_m, -0.12_m, 0_deg, true);
    backupFromGoal();
    //robot::drive->controller->driveForDistance(0.27_m);
    //pros::delay(150);
    robot::conveyor->moveBoth(1.0);
    pros::delay(1100);
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
    backout(700);
    
    // Move to third goal while out-taking
    robot::intake->spinOut(1.0);
    robot::drive->controller->strafeToPose({1.97_m, -0.40_m, -45_deg}, 2000);
    robot::intake->stop();
    // TODO(louis): ASYNC ACTION

    // Intake the ball in front of the third goal & score it
    robot::intake->spinIn(1.0);
    robot::drive->controller->driveForDistance(0.42_m, 800);
    // TODO(louis): ASYNC ACTION
    pros::delay(350);
    robot::intake->stop();

    robot::drive->model->xArcade(0.0, 0.7, 0.0);
    pros::delay(400);
    backupFromGoal();
    robot::conveyor->moveBoth(1.0);
    pros::delay(1000);
    robot::conveyor->stopAll();

    // Backup and out-take
    robot::intake->spinOut(1.0);
    robot::drive->model->xArcade(0.0, -1.0, 0.0);
    pros::delay(800);
    robot::intake->stop();
    robot::drive->model->stop();
}

void goingMid() {
    reset(0_m, 0_m, 0_deg);
    rightSide1(true, false);
    // Backout to second goal
    robot::intake->spinIn(1.0);
    robot::drive->model->xArcade(0.0, -0.8, 0.0);
    robot::conveyor->startIndexing(robot::Conveyor::ControlMode::StoreBall);
    robot::conveyor->moveBoth(1.0);
    robot::conveyor->waitUntilStored(robot::Conveyor::Position::Top);
    robot::conveyor->stopAll();
    robot::intake->stop();
    robot::drive->model->stop();
    robot::drive->controller->strafeToPose({-0.73_m, -1.31_m, -225_deg}, 4000);
    robot::drive->model->xArcade(0.0, 0.4, 0.0);
    pros::delay(100);
    robot::intake->spinOut(1.0);
    robot::conveyor->startIndexing(robot::Conveyor::ControlMode::WaitUntilEmpty);
    robot::conveyor->moveBoth(-1.0);
    robot::conveyor->waitUntilEmpty(robot::Conveyor::Position::Top);
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
