#include "main.h"
#include "autonomous.hpp"
#include "constants.hpp"
#include "robot.hpp"
#include "libraidzero/planner/profilePlanner.hpp"
#include "robot/drive.hpp"

#include <exception>
#include <string>

void autonomous() {
    /*robot::drive::generatePath({
        {0_m, 0_m, 0_deg},
        {3.0_ft, 3.0_ft, 0_deg}
    }, "test");
    robot::drive::resetEncoders();

    robot::drive::followPath("test", true);*/
    Controller master {ControllerId::master};
    robot::drive::resetEncoders();
    robot::drive::controller->setState({0_m, 0_m, 0_deg});
    robot::drive::controller->setMaxVoltage(0.7 * 12000);
    // robot::drive::controller->driveForDistance(0.5_m);
    // std::cout << robot::drive::controller->getState().str() << std::endl;
    // std::cout << robot::drive::controller->getState().str() << std::endl;
    //robot::drive::controller->turnToAngle(90_deg);
    // robot::drive::controller->driveToPoint({60_cm, 0_cm});
    // pros::delay(300);
    // robot::drive::controller->turnToAngle(180_deg);
    // pros::delay(300);
    // robot::drive::controller->driveToPoint({30_cm, 0_cm});
    // pros::delay(300);
    auto profile = planner::ProfilePlanner::generatePath(
        {
            {0_m, 0_m, 0_deg},
            {0.45_m, 0.45_m, 0_deg},
            {0_m, 0.9_m, 0_deg}
        }, planner::PlannerConfig{DRIVE_MAX_VEL, DRIVE_MAX_ACCEL, 0.0},
        true
    );
    Trajectory traj{Trajectory::profileToStates(profile)};

    robot::drive::controller->followTrajectoryAsync(traj);
    robot::drive::controller->waitUntilSettled();
    pros::delay(300);
    
    // robot::drive::controller->strafeToPoseAsync({-0.4_m, 0.4_m, 90_deg});
    // robot::drive::controller->waitUntilSettled();
    // pros::delay(300);
    // robot::drive::controller->strafeToPoseAsync({0_m, 0.4_m, -90_deg});
    // robot::drive::controller->waitUntilSettled();
    // pros::delay(300);
    // robot::drive::controller->strafeToPoseAsync({-0.4_m, 0_m, 90_deg});
    // robot::drive::controller->waitUntilSettled();
    // pros::delay(300);
    // robot::drive::controller->strafeToPoseAsync({0_m, 0_m, 0_deg});
    // robot::drive::controller->waitUntilSettled();

    // pros::delay(500);
    // robot::drive::controller->strafeToPointAsync({0_m, 0.5_m});
    // robot::drive::controller->waitUntilSettled();
    // pros::delay(500);
    // robot::drive::controller->strafeToPointAsync({-0.5_m, 0_m});
    // robot::drive::controller->waitUntilSettled();
    // pros::delay(500);
    // robot::drive::controller->strafeToPointAsync({0_m, 0_m});
    // robot::drive::controller->waitUntilSettled();
    // pros::delay(500);
    // robot::drive::controller->turnToAngle(45_deg);
    // pros::delay(500);
    // robot::drive::controller->strafeToPointAsync({0_m, 1.0_m});
    // robot::drive::controller->waitUntilSettled();
    // // robot::drive::controller->driveForDistance(0.4_m);
    // master.setText(0, 0, std::to_string(robot::drive::controller->getState().x.convert(meter)));
    /*robot::drive::profileFollower->generatePath({
        {0_m, 0_m, 0_deg},
        {0.45_m, 0.45_m, 0_deg}
    }, "forward");
    robot::drive::profileFollower->setTarget("forward");
    robot::drive::profileFollower->flipDisable(false);
    robot::drive::profileFollower->waitUntilSettled();*/

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