#pragma once

#include "libraidzero/planner/profileStructs.hpp"
#include "libraidzero/planner/hermiteInterpolator.hpp"
#include "libraidzero/planner/polynomialFunction.hpp"

#include <vector>

namespace planner {

    class ProfilePlanner {
    private:
        struct SplinePair {
            HermiteInterpolator x;
            HermiteInterpolator y;

            SplinePair() : x{}, y{} {}
        };

        struct QueryData {
            const std::vector<double> cumulativeWaypointDistances;
            double totalWaypointDistance;
            int queryCount;

            QueryData(const std::vector<double>&& cumulativeWaypointDistances, 
                double totalWaypointDistance, int queryCount) :
                cumulativeWaypointDistances{cumulativeWaypointDistances},
                totalWaypointDistance{totalWaypointDistance},
                queryCount{queryCount} {}
        };

        static std::vector<double> calculateCumulativeDistances(
            const std::vector<Waypoint>& waypoints);
    public:
        /**
         * Number of query points per meter of approximate distance.
         *
         * Increasing this number results in more query points and greater
         * accuracy for path generation.
         */
        static constexpr double QUERY_INTERVAL = 0.01;

        /**
         * Generates a path that passes through the given waypoints on the field.
         *
         * The returned path contains data which can be passed to motion profile to execute the path.
         * The robot will accelerate at a constant rate, then reach a constant cruise velocity, then
         * decelerate at the same rate. In other words, the velocity vs time graph shows an isosceles
         * trapezoid.
         *
         * @param waypoints points that the path should pass through
         * @param cruiseVelocity the target constant cruise velocity of the robot in m/s
         * @param targetAcceleration the target constant acceleration (and deceleration) of the robot 
         * in m/s^2
         * @param initialVelocity the initial velocity of the robot in m/s
         * @return an array of points on the path
         */
        static planner::MotionProfile generatePath(const std::vector<Waypoint>& waypoints,
            double cruiseVelocity, double targetAcceleration, double initialVelocity = 0.0);

        static QueryData getQueryData(const std::vector<Waypoint>& waypoints);

        static SplinePair calculateSplines(
            const std::vector<Waypoint>&, const QueryData& queryData);

        static void calculatePathPoints(MotionProfile& profile,
            double cruiseVelocity, double targetAcceleration, double initialVelocity,
            const SplinePair& splines, const QueryData& queryData);

        static std::vector<double> queryDeriv(const planner::PolynomialFunction& p, 
            const QueryData& queryData);
        static std::vector<double> queryCoord(const planner::HermiteInterpolator& h,
            const QueryData& queryData);
    };

}