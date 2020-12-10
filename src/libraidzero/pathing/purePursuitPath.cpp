#include "libraidzero/pathing/purePursuitPath.hpp"
#include <algorithm>
#include <cmath>

PurePursuitPath::PurePursuitPath(std::vector<Pose2d> iwaypoints, 
    QLength ispacing, Constraints iconstraints
) : points{injectPoints(std::move(iwaypoints), ispacing)},
    constraints{iconstraints}
{

}

const PurePursuitPath::Point& PurePursuitPath::getPoint(int iindex) const {
    return points.at(iindex);
}

const PurePursuitPath::Point& PurePursuitPath::getLastPoint() const {
    return points.back();
}

int PurePursuitPath::getPointCount() const {
    return points.size();
}

std::vector<PurePursuitPath::Point> PurePursuitPath::injectPoints(
    std::vector<Pose2d> iwaypoints, QLength ispacing
) {
    std::vector<Point> points;
    int waypointCount = iwaypoints.size();
    // Iterate through all the waypoints / line segments
    for (int i = 0; i < waypointCount - 1; ++i) {
        auto dirVec = iwaypoints[i + 1].translation() - iwaypoints[i].translation();
        int numberOfPoints = std::ceil((dirVec.norm() / ispacing).getValue());
        std::cout << "Number of points that fit: " << numberOfPoints << std::endl;
        dirVec = dirVec / dirVec.norm().convert(meter) * ispacing.convert(meter);
        for (int j = 0; j < numberOfPoints; ++j) {
            points.push_back({{
                iwaypoints[i].translation() + dirVec * j, 
                Rotation2d{lerp(
                    iwaypoints[i].angle(), 
                    iwaypoints[i + 1].angle(),
                    j / static_cast<double>(numberOfPoints)
                )}
            }, 0.0, 0.0});
        }
    }
    points.push_back({iwaypoints[waypointCount - 1], 0.0, 0.0});
    return points;
}

double PurePursuitPath::distance(const Pose2d& p1, const Pose2d& p2) {
    return p1.translation().distance(p2.translation()).convert(meter);
}

void PurePursuitPath::smoothen(double ia, double ib, double itolerance) {
    std::vector<Point> newPoints = points;
    int pointCount = points.size();

    double change = itolerance;
    while (change >= itolerance) {
        change = 0.0;
        for (int i = 1; i < pointCount - 1; ++i) {
            double currentX = newPoints[i].pose.x().convert(meter);
            double newX = currentX + ia * (points[i].pose.x().convert(meter) - currentX) + 
                ib * (newPoints[i - 1].pose.x().convert(meter) + newPoints[i + 1].pose.x().convert(meter) - 2 * currentX);
            change += std::abs(currentX - newX);
            double currentY = newPoints[i].pose.y().convert(meter);
            double newY = currentY + ia * (points[i].pose.y().convert(meter) - currentY) + 
                ib * (newPoints[i - 1].pose.y().convert(meter) + newPoints[i + 1].pose.y().convert(meter) - 2 * currentY);
            newPoints[i].pose = {newX * meter, newY * meter, newPoints[i].pose.angle()};
            change += std::abs(currentY - newY);
        }
    }
    points = newPoints;
}

void PurePursuitPath::fillPointInformation() {
    const int pointCount = points.size();
    for (int i = 1; i < pointCount - 1; ++i) { // Don't care about end points
        // Distance
        points[i].distanceFromStart = points[i - 1].distanceFromStart + 
            distance(points[i].pose, points[i - 1].pose);
        // Curvature
        const auto& prevPoint = points[i - 1].pose;
        const auto& nextPoint = points[i + 1].pose;
        const auto& currentPoint = points[i].pose;
        double x1 = currentPoint.x().convert(meter);
        double y1 = currentPoint.y().convert(meter);
        double x2 = prevPoint.x().convert(meter);
        double y2 = prevPoint.y().convert(meter);
        double x3 = nextPoint.x().convert(meter);
        double y3 = nextPoint.y().convert(meter);
        if (std::abs(x1 - x2) < 0.0001) {
            if (std::abs(x1 - x3) < 0.0001) {
                points[i].curvature = 0.0;
                continue;
            }
            // Avoid division by zero
            x1 += 0.001;
        }
        double k1 = 0.5 * (x1 * x1 + y1 * y1 - x2 * x2 - y2 * y2) / (x1 - x2);
        double k2 = (y1 - y2) / (x1 - x2);
        double b = 0.5 * (x2 * x2 - 2 * x2 * k1 + y2 * y2 - x3 * x3 + 2 * x3 * k1 - y3 * y3) / (x3 * k2 - y3 + y2 - x2 * k2);
        double a = k1 - k2 * b;
        double r = std::sqrt((x1 - a) * (x1 - a) + (y1 - b) * (y1 - b));
        double curvature = 1 / r;
        if (std::isnan(curvature)) {
            curvature = 0.0;
        }
        points[i].curvature = curvature;
    }
    points.back().distanceFromStart = points[pointCount - 2].distanceFromStart + 
        distance(points.back().pose, points[pointCount - 2].pose);
    // Target velocities
    for (auto& point : points) {
        point.targetVelocity = std::min(constraints.maxVelocity, constraints.kTurn / point.curvature);
    }
    // Acceleration-constrained target velocities
    points.back().targetVelocity = 0.0;
    for (int i = pointCount - 2; i >= 0; --i) {
        double d = distance(points[i + 1].pose, points[i].pose);
        double nextPointVelocity = points[i + 1].targetVelocity;
        points[i].targetVelocity = std::min(
            points[i].targetVelocity,
            std::sqrt(nextPointVelocity * nextPointVelocity + 2 * constraints.maxAcceleration * d)
        );
    }
}

PurePursuitPath::Point PurePursuitPath::getClosestPoint(const Pose2d& ipose) {
    double closestDist = std::numeric_limits<double>::max();
    Point closestPoint = points.back();
    for (auto it = points.rbegin(); it != points.rend(); ++it) {
        double d = distance((*it).pose, ipose);
        if (d < closestDist) {
            closestPoint = *it;
            closestDist = d;
        }
    }
    return closestPoint;
}
