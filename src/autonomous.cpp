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

void rightSide() {
    robot::drive::resetEncoders();
    robot::drive::model->resetGyro(180_deg);
    robot::drive::controller->setState({0_m, 0_m, 180_deg});
    robot::drive::controller->setMaxVoltage(0.9 * 12000);

    robot::drive::controller->strafeToPoseAsync({-0.57_m, 0.57_m, 180_deg});
    robot::drive::controller->waitUntilSettled(2000);
    pros::delay(100);
    robot::drive::controller->strafeToPoseAsync({-0.6_m, 0.2_m, 180_deg});
    pros::delay(200);
    robot::conveyor::moveUp(1.0, robot::conveyor::Position::Top);
    robot::conveyor::moveUp(1.0, robot::conveyor::Position::Bottom);
    robot::drive::controller->waitUntilSettled(1300);
    robot::conveyor::stop();

    robot::drive::controller->strafeToPoseAsync({-0.6_m, 0.6_m, 225_deg});
    robot::intake::spinOut(1.0);
    robot::drive::controller->waitUntilSettled(1300);
    robot::intake::stop();

    robot::drive::controller->strafeToPoseAsync({0.3_m, 0.6_m, 225_deg});
    robot::drive::controller->waitUntilSettled(3000);
    pros::delay(200);
    robot::drive::controller->strafeToPoseAsync({0.8_m, 0.1_m, 225_deg});
    robot::intake::spinIn(1.0);
    pros::delay(1000);
    robot::intake::stop();
    robot::conveyor::moveUp(1.0, robot::conveyor::Position::Top);
    robot::conveyor::moveUp(1.0, robot::conveyor::Position::Bottom);
    robot::drive::controller->waitUntilSettled(1600);
    robot::conveyor::stop();
    robot::intake::spinOut(1.0);
    robot::drive::controller->strafeToPoseAsync({0.2_m, 0.6_m, 225_deg});
    robot::drive::controller->waitUntilSettled(500);
    robot::intake::stop();
}

void autonomous() {
    /*robot::drive::generatePath({
        {0_m, 0_m, 0_deg},
        {3.0_ft, 3.0_ft, 0_deg}
    }, "test");
    robot::drive::resetEncoders();

    robot::drive::followPath("test", true);*/
    rightSide();
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