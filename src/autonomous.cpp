#include "main.h"
#include "autonomous.hpp"
#include "constants.hpp"
#include "robot.hpp"

#include <exception>

void autonomous() {
    robot::drive::generatePath({
        {0_m, 0_m, 0_deg},
        {3.0_ft, 3.0_ft, 0_deg}
    }, "test");
    robot::drive::resetEncoders();
    robot::drive::followPath("test", true);
}