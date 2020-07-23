#pragma once

#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include <optional>
#include <vector>

namespace planner {

    struct PathPoint {
        /**
         * X coordinate of the path point on the spline.
         */
        double x;

        /**
         * Y coordinate of the path point on the spline.
         */
        double y;

        /**
         * Position of the robot in meters.
         *
         * <p>Note: position means the distance the robot has traveled by the time it reaches this
         * point, not the 2-dimensional x and y coordinates of the robot.
         */
        double position;

        /**
         * Velocity of the robot in m/s.
         */
        double velocity;

        /**
         * Acceleration of the robot in m/s^2
         */
        double acceleration;

        /**
         * Time it takes for the robot to go from the previous point to the current point in seconds.
         *
         * <p>This is 0 for the first point in the path.
         */
        double time;

        /**
         * Angle heading of the robot in degrees.
         */
        double angle;
    };

    /**
     * Simple tuple-like data class with x y coordinates and optional angle.
     */
    struct Waypoint {
        /**
         * The x-coordinate.
         */
        double x;

        /**
         * The y-coordinate.
         */
        double y;

        /**
         * The angle in degrees, if provided.
         */
        std::optional<double> angle;

        /**
         * Constructs a Point object.
         *
         * @param x the x-coordinate
         * @param y the y-coordinate
         */
        Waypoint(double x, double y) :
            x{x}, y{y}, angle{}
        {}

        /**
         * Constructs a Point object.
         *
         * @param x the x-coordinate
         * @param y the y-coordinate
         * @param a the angle in degrees
         */
        Waypoint(double x, double y, double angle) :
            x{x}, y{y}, angle{angle}
        {}
    };

    /**
     * Simple tuple-like data class with x y coordinates and optional angle.
     */
    struct UnitableWaypoint {
        /**
         * The x-coordinate.
         */
        okapi::QLength x;

        /**
         * The y-coordinate.
         */
        okapi::QLength y;

        /**
         * The angle in degrees, if provided.
         */
        std::optional<okapi::QAngle> angle;

        /**
         * Constructs a Point object.
         *
         * @param x the x-coordinate
         * @param y the y-coordinate
         */
        UnitableWaypoint(okapi::QLength x, okapi::QLength y) :
            x{x}, y{y}, angle{}
        {}

        /**
         * Constructs a Point object.
         *
         * @param x the x-coordinate
         * @param y the y-coordinate
         * @param a the angle in degrees
         */
        UnitableWaypoint(okapi::QLength x, okapi::QLength y, okapi::QAngle angle) :
            x{x}, y{y}, angle{angle}
        {}
    };

    struct MotionProfile {
        std::vector<PathPoint> pathPoints{};
        double totalLength = 0.0;
        double totalTime = 0.0;
    };

    /*
     * @param cruiseVelocity the target constant cruise velocity of the robot in m/s
     * @param targetAcceleration the target constant acceleration (and deceleration) of the robot 
     * in m/s^2
     * @param initialVelocity the initial velocity of the robot in m/s
     */
    struct PlannerConfig {
        double cruiseVelocity = 0.0;
        double targetAcceleration = 0.0;
        double initialVelocity = 0.0;
    };

}