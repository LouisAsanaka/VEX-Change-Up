#include "libraidzero/pathing/purePursuitPath.hpp"

PurePursuitPath::PurePursuitPath(std::vector<Pose2d> iwaypoints, QLength ispacing)
    : points{injectPoints(std::move(iwaypoints), ispacing)}
{

}

Pose2d PurePursuitPath::getPoint(int iindex) const {
    return points.at(iindex);
}

std::vector<Pose2d> PurePursuitPath::injectPoints(std::vector<Pose2d> iwaypoints, 
    QLength ispacing)
{
    std::vector<Pose2d> points;
    int waypointCount = iwaypoints.size();
    // Iterate through all the waypoints / line segments
    for (int i = 0; i < waypointCount - 1; ++i) {
        auto dirVec = iwaypoints[i + 1].translation() - iwaypoints[i].translation();
        int numberOfPoints = std::ceil((dirVec.norm() / ispacing).getValue());
        std::cout << "Number of points that fit: " << numberOfPoints << std::endl;
        dirVec = dirVec / dirVec.norm().convert(meter) * ispacing.convert(meter);
        for (int j = 0; j < numberOfPoints; ++j) {
            points.push_back({
                iwaypoints[i].translation() + dirVec * j, 
                Rotation2d{lerp(
                    iwaypoints[i].angle(), 
                    iwaypoints[i + 1].angle(),
                    j / static_cast<double>(numberOfPoints)
                )}
            });
        }
    }
    points.push_back(iwaypoints[waypointCount - 1]);
    return points;
}

void PurePursuitPath::smoothen(double ia, double ib, double itolerance) {
    std::vector<Pose2d> newPoints = points;
    int pointCount = points.size();

    double change = itolerance;
    while (change >= itolerance) {
        change = 0.0;
        for (int i = 1; i < pointCount - 1; ++i) {
            auto currentX = newPoints[i].x();
            auto newX = ia * (points[i].x() - currentX) + 
                ib * (newPoints[i - 1].x() + newPoints[i + 1].x() - 2 * currentX);
            change += (currentX - newX).abs().getValue();
            auto currentY = newPoints[i].y();
            auto newY = ia * (points[i].y() - currentY) + 
                ib * (newPoints[i - 1].y() + newPoints[i + 1].y() - 2 * currentY);
            change += (currentY - newY).abs().getValue();
        }
    }
    points = newPoints;
}