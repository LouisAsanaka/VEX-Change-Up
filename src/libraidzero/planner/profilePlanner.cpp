#include "libraidzero/planner/profilePlanner.hpp"
#include "libraidzero/planner/profileStructs.hpp"

#include <vector>
#include <cmath>
#include <iostream>
#include <chrono>

#define M_PI 3.14159265358979323846

planner::MotionProfile planner::ProfilePlanner::generatePath(
    const std::vector<Waypoint>& waypoints,
    const PlannerConfig& config)
{
    auto queryData = getQueryData(waypoints);
    auto splines = calculateSplines(waypoints, queryData);

    auto profile = MotionProfile{};

    profile.pathPoints.reserve(queryData.queryCount);
    for (int i = 0; i < queryData.queryCount; i++) {
        profile.pathPoints.emplace_back();
    }

    calculatePathPoints(profile,
        config.cruiseVelocity, config.targetAcceleration, config.initialVelocity,
        splines, queryData);

    auto xQueries = queryCoord(splines.x, queryData);
    auto yQueries = queryCoord(splines.y, queryData);

    for (int i = 0; i < queryData.queryCount; i++) {
        profile.pathPoints[i].x = xQueries[i];
        profile.pathPoints[i].y = yQueries[i];
    }

    return profile;
}

planner::ProfilePlanner::QueryData planner::ProfilePlanner::getQueryData(
    const std::vector<planner::Waypoint>& waypoints)
{
    auto distances = calculateCumulativeDistances(waypoints);
    double totalDistance = distances.back();
    auto queryData = QueryData{
        distances,
        totalDistance,
        static_cast<int>(std::ceil(totalDistance / QUERY_INTERVAL)) + 1
    };
    //std::cout << "Using " << queryData.queryCount << " points for generation." << std::endl;
    return queryData;
}

planner::ProfilePlanner::SplinePair planner::ProfilePlanner::calculateSplines(
    const std::vector<planner::Waypoint>& waypoints, const QueryData& queryData)
{
    SplinePair splinePair{};
    for (int i = 0; i < waypoints.size(); i++) {
        double waypointDistance = queryData.cumulativeWaypointDistances.at(i);
        double waypointX = waypoints[i].x;
        double waypointY = waypoints[i].y;

        if (waypoints[i].angle.has_value()) {
            double angle = waypoints[i].angle.value() * M_PI / 180;
            splinePair.x.addSamplePoint(waypointDistance, std::vector<double>{
                waypointX, std::cos(angle)
            });
            splinePair.y.addSamplePoint(waypointDistance, std::vector<double>{
                waypointY, std::sin(angle)
            });
        } else {
            splinePair.x.addSamplePoint(waypointDistance, std::vector<double>{
                waypointX
            });
            splinePair.y.addSamplePoint(waypointDistance, std::vector<double>{
                waypointY
            });
        }
    }
    return splinePair;
}

void planner::ProfilePlanner::calculatePathPoints(MotionProfile& profile,
    double cruiseVelocity, double targetAcceleration, double initialVelocity,
    const SplinePair& splines, const QueryData& queryData) 
{
    auto& path = profile.pathPoints;
    int pathSize = path.size();

    auto dxQueries = queryDeriv(splines.x.getPolynomials()[0].derivative(), queryData);
    auto dyQueries = queryDeriv(splines.y.getPolynomials()[0].derivative(), queryData);

    // The angle for each point is calculated using arctan of the ratio between dy and dx.
    // Since atan2 wraps the angle to be within -180 to 180 (because it has no way of knowing
    // the actual angle of the robot), we estimate the real unwrapped angle by looking at if the
    // current angle would be closer to the previous angle after adding or subtracting 360. This
    // is fine because the robot can't turn more than 180 degrees in between two data points on
    // the path. The angle for the first point should be given.
    for (int i = 0; i < pathSize; i++) {
        path[i].angle = std::atan2(dyQueries[i], dxQueries[i]) * 180 / M_PI;
        if (i > 0) {
            while (path[i].angle - path[i - 1].angle > 180) {
                path[i].angle -= 360;
            }
            while (path[i].angle - path[i - 1].angle < -180) {
                path[i].angle += 360;
            }
        }
    }

    // Position is calculated with a running total of arc lengths with Riemann sum. Arclength
    // formula integrate(hypot(dy/dt, dx/dt)*dt) where dt is QUERY_INTERVAL. Rectangles are
    // centered at each querypoint, so the cumulative area under curve is half a rectangle each
    // from the last point and the current point. The interval for the last pathpoint is shorter
    // as it is the last waypoint, and QUERY_INTERVAL does not divide into the full path length.
    for (int i = 1; i < pathSize; i++) {
        double interval = QUERY_INTERVAL;
        if (i == pathSize - 1) {
            double remainder = std::fmod(queryData.totalWaypointDistance, QUERY_INTERVAL);
            if (remainder != 0) {
                interval = remainder;
            }
        }
        path[i].position = 0.5 * interval * (std::hypot(dxQueries[i], dyQueries[i])
            + std::hypot(dxQueries[i - 1], dyQueries[i - 1])) + path[i - 1].position;
    }
    double totalDistance = path[pathSize - 1].position;

    // Calculations for velocity and time are in three separate stages for the acceleration,
    // constant velocity, and deceleration parts respectively.
    {
        bool reachesCruiseVelocity = false;

        double v0 = initialVelocity;
        double v02 = v0 * v0;
        // First stage: we find velocity and time for data points assuming constant acceleration
        // up until either (1) we reach the cruise velocity, or (2) we have not reached cruise
        // velocity and we are halfway through the path already, in which case we must start
        // decelerating.
        int i;
        // TODO: totalDistance / 2 comparison should not be needed
        for (i = 0; path[i].position <= totalDistance / 2; i++) {
            // v^2 = v0^2 + 2 a x
            // v = sqrt(v0^2 + 2 a x)
            double velocity = std::sqrt(v02 + 2 * targetAcceleration * path[i].position);
            // v^2 = v0^2 + 2 a x
            // x = (v^2 - v0^2) / (2 a)
            // v = 0, a < 0
            // The expected distace needed to decelerate with the current velocity
            double decelerationDistance = velocity * velocity / (2 * targetAcceleration);
            if (decelerationDistance > totalDistance - path[i].position) {
                break;
            }

            if (velocity > cruiseVelocity) {
                reachesCruiseVelocity = true;
                break;
            }
            path[i].velocity = velocity;
            // x = v0 t + (1/2) a t^2
            // t = (sqrt(2 a x + v0^2) - v0) / a
            path[i].time = (std::sqrt(2 * targetAcceleration * path[i].position + v02) - v0)
                / targetAcceleration;
            //path[i].time = Math.sqrt(2 * path[i].position / targetAcceleration);
        }
        // Third stage for velocity: we do the same thing as the first stage, but working
        // backwards instead. We stop when either (1) we reach the cruise velocity, or (2) we
        // reach where stage 1 stopped, in which case there is no constant velocity part.
        int j;
        for (j = pathSize - 1; j >= i; j--) {
            double distanceFromEnd = totalDistance - path[j].position;
            double velocity = std::sqrt(2 * targetAcceleration * distanceFromEnd);
            if (velocity > cruiseVelocity) {
                break;
            }
            path[j].velocity = velocity;
        }
        // v = v0 + a t
        // t = (v - v0) / a
        double cruiseStartTime = (cruiseVelocity - v0) / targetAcceleration;
        // v^2 = v0^2 + 2 a x
        // x = (v^2 - v0^2) / (2 a)
        double cruiseStartPosition = (cruiseVelocity * cruiseVelocity - v02) / (2 * targetAcceleration);
        int k;
        // Second stage: we calculate velocity and time in between the end of first stage and
        // start of third stage. If we never reach the cruise velocity, this loop never runs.
        for (k = i; k <= j; k++) {
            path[k].velocity = cruiseVelocity;
            // t = t0 + delta_t
            // v = delta_x / delta_t
            // delta_t = delta_x / v
            // delta_x = x - x0
            // t = t0 + (x - x0) / v
            path[k].time = cruiseStartTime
                + (path[k].position - cruiseStartPosition) / cruiseVelocity;
        }
        int endOfCruiseIndex = k;
        int endOfAccelIndex = i;
        double decelerateStartPosition = reachesCruiseVelocity
            // The next position at the end of the cruising period is the start
            ? path[endOfCruiseIndex].position
            : path[endOfAccelIndex].position;
        double decelerateInitialVelocity = reachesCruiseVelocity
            ? cruiseVelocity
            : path[endOfAccelIndex].velocity;
        double decelerateStartTime = reachesCruiseVelocity
            ? path[endOfCruiseIndex - 1].time
            : path[endOfAccelIndex - 1].time;

        double decelerationDistance = decelerateInitialVelocity * decelerateInitialVelocity 
            / (2 * targetAcceleration);
        double endingPosition = decelerateStartPosition + decelerationDistance;

        // Third stage for time: calculate from end of stage 2 to end of path
        for (; k < pathSize; k++) {
            // x = x0 + v0 t + (1/2) (-a) t^2
            // (1/2) (-a) t^2 + v0 t + (x0 - x) = 0
            // t = (-v0 +- sqrt(v0^2 - 4 ((1/2) (-a)) (x0 - x))) / (2 ((1/2) (-a)))
            // t = (-v0 +- sqrt(v0^2 - 2 (-a) (x0 - x))) / -a
            double pos = path[k].position;
            if (pos > endingPosition) {
                path[k].time = decelerateStartTime + decelerateInitialVelocity / targetAcceleration;
            } else {
                path[k].time = decelerateStartTime
                    + (-decelerateInitialVelocity
                        + std::sqrt(decelerateInitialVelocity * decelerateInitialVelocity
                            - 2 * -targetAcceleration
                            * (decelerateStartPosition - path[k].position)))
                    / -targetAcceleration;
            }
        }
    }

    /*for (int l = 0; l < pathSize; ++l) {
        std::cout << "Index " << l << std::endl;
        std::cout << "  Time = " << path[l].time << " s" << std::endl;
        std::cout << "  Pos  = " << path[l].position << " m" << std::endl;
        std::cout << "  Vel  = " << path[l].velocity << " m/s" << std::endl;
        std::cout << "  Ang  = " << path[l].angle << " deg" << std::endl;
    }*/

    profile.totalTime = path.back().time;
    profile.totalLength = totalDistance;

    // Calculate the time differences (i.e. make the times not cumulative)
    for (int i = path.size() - 1; i > 0; i--) {
        path[i].time -= path[i - 1].time;
    }
}

std::vector<double> planner::ProfilePlanner::queryDeriv(const planner::PolynomialFunction& p, 
    const QueryData& queryData) 
{
    std::vector<double> queries = std::vector<double>(queryData.queryCount);
    for (int i = 0; i < queryData.queryCount - 1; i++) {
        queries[i] = p.value(i * QUERY_INTERVAL);
    }
    queries[queryData.queryCount - 1] = p.value(queryData.totalWaypointDistance);
    return queries;
}

std::vector<double> planner::ProfilePlanner::queryCoord(const planner::HermiteInterpolator& h,
    const QueryData& queryData)
{
    std::vector<double> queries = std::vector<double>(queryData.queryCount);
    for (int i = 0; i < queryData.queryCount - 1; i++) {
        queries[i] = h.value(i * QUERY_INTERVAL)[0];
    }
    queries[queryData.queryCount - 1] = h.value(queryData.totalWaypointDistance)[0];
    return queries;
}

std::vector<double> planner::ProfilePlanner::calculateCumulativeDistances(
    const std::vector<Waypoint>& waypoints)
{
    int size = waypoints.size();
    auto cumulativeDistances = std::vector<double>(size);
    cumulativeDistances[0] = 0.0;
    for (int i = 1; i < size; i++) {
        cumulativeDistances[i] = cumulativeDistances[i - 1]
            + std::hypot(waypoints[i].x - waypoints[i - 1].x,
                waypoints[i].y - waypoints[i - 1].y);
    }
    return cumulativeDistances;
}
