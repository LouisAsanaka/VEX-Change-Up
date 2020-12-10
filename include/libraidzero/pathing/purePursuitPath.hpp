#pragma once

#include "libraidzero/geometry/pose2d.hpp"
#include "libraidzero/util/mathUtil.hpp"

#include "okapi/api.hpp"

#include <vector>

using namespace okapi;

struct PurePursuitPath {
public:
    struct Point {
        Pose2d pose;
        double distanceFromStart = 0.0;
        double curvature = 0.0;
        double targetVelocity = 0.0;
    };

    struct Constraints {
        double maxVelocity;
        double maxAcceleration;
        double kTurn;

        constexpr Constraints(double imaxVelocity, double imaxAcceleration, double ikTurn = 3.0) : 
            maxVelocity{imaxVelocity}, maxAcceleration{imaxAcceleration}, kTurn{ikTurn} {}
    };

    std::vector<Point> points;
    Constraints constraints;

    PurePursuitPath(std::vector<Pose2d> iwaypoints, QLength ispacing, 
        Constraints iconstraints);

    const Point& getPoint(int iindex) const;
    const Point& getLastPoint() const;
    int getPointCount() const;

    void smoothen(double ia, double ib, double itolerance);
    void fillPointInformation();

    Point getClosestPoint(const Pose2d& ipose);

    static double distance(const Pose2d& p1, const Pose2d& p2);
private:
    static std::vector<Point> injectPoints(
        std::vector<Pose2d> iwaypoints, QLength spacing);  
};
