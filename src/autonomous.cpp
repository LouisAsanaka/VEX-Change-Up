#include "main.h"
#include "autonomous.hpp"
#include "constants.hpp"
#include "robot.hpp"
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
    // robot::drive::controller->driveForDistance(0.5_m);
    // std::cout << robot::drive::controller->getState().str() << std::endl;
    robot::drive::controller->turnAngle(90_deg);
    std::cout << robot::drive::controller->getState().str() << std::endl;
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