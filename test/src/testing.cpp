#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include "doctest.h"
#include <cmath>
#include "okapi/api/units/QLength.hpp"
#include "libraidzero/planner/profileStructs.hpp"
#include "libraidzero/planner/profilePlanner.hpp"

TEST_CASE("testing stuff") {
    using namespace okapi;
    std::vector<planner::Waypoint> waypoints;
    waypoints.emplace_back(0.0, 0.0, 45.0);
    waypoints.emplace_back(2.0, 2.0, 45.0);
    planner::MotionProfile mp = planner::ProfilePlanner::generatePath(
        waypoints,
        planner::PlannerConfig{1.0, 1.0, 0.0},
        true
    );
    CHECK(mp.totalLength == doctest::Approx(std::sqrt(2 * 2 + 2 * 2)));
    CHECK((2_m).convert(meter) == 2);
}