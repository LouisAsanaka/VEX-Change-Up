#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include <cmath>
#include <iostream>
#include "okapi/api/units/QLength.hpp"
#include "libraidzero/planner/profileStructs.hpp"
#include "libraidzero/planner/profilePlanner.hpp"
#include "libraidzero/trajectory/trajectory.hpp"

TEST_CASE("testing stuff") {
    using namespace okapi;
    planner::MotionProfile mp = planner::ProfilePlanner::generatePath(
        {
            {0_m, 0_m, 90_deg},
            {0.45_m, 0.45_m, 90_deg},
            {0_m, 0.9_m, 90_deg}
        },
        planner::PlannerConfig{1.0, 1.0, 0.0},
        true
    );
    // Trajectory traj{Trajectory::profileToStates(mp)};
    
    // std::cout << "Total time: " << traj.getTotalTime() << std::endl;
    // for (double i = 0; i <= traj.getTotalTime() + 1; i += 0.1) {
    //     auto pose = traj.sample(i).pose;
    //     std::cout 
    //         << pose.translation().x().convert(okapi::meter) << ","
    //         << pose.translation().y().convert(okapi::meter) << ","
    //         << i
    //         << std::endl;
    // }
    CHECK((2_m).convert(meter) == 2);
}