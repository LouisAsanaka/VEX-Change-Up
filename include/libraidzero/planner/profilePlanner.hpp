#pragma once

#include "libraidzero/planner/profileStructs.hpp"
#include "libraidzero/planner/hermiteInterpolator.hpp"
#include "libraidzero/planner/polynomialFunction.hpp"

#include <utility>
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
            std::vector<double> cumulativeWaypointDistances;
            double totalWaypointDistance;
            int queryCount;

            QueryData(std::vector<double> cumulativeWaypointDistances, 
                double totalWaypointDistance, int queryCount) :
                cumulativeWaypointDistances{std::move(cumulativeWaypointDistances)},
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
         * @param config configuration for the profile
         * @param fillCoords whether to fill in x y coordinates or not
         * @return an array of points on the path
         */
        static planner::MotionProfile generatePath(const std::vector<Waypoint>& waypoints,
            const PlannerConfig& config, bool fillCoords = false);

        static planner::MotionProfile generatePath(
            std::initializer_list<UnitableWaypoint> waypoints,
            const planner::PlannerConfig& config
        );

        static QueryData getQueryData(const std::vector<Waypoint>& waypoints);

        static SplinePair calculateSplines(
            const std::vector<Waypoint>& waypoints, const QueryData& queryData);

        static void calculatePathPoints(MotionProfile& profile,
            double cruiseVelocity, double targetAcceleration, double initialVelocity,
            const SplinePair& splines, const QueryData& queryData);

        static std::vector<double> queryDeriv(const planner::PolynomialFunction& p, 
            const QueryData& queryData);
        static std::vector<double> queryCoord(const planner::HermiteInterpolator& h,
            const QueryData& queryData);
    };

}