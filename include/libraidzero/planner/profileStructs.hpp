#pragma once

#include <optional>
#include <vector>

namespace planner {

    struct PathPoint {
        double x;

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

    struct MotionProfile {
        std::vector<PathPoint> pathPoints{};
        double totalLength = 0.0;
        double totalTime = 0.0;
    };

}