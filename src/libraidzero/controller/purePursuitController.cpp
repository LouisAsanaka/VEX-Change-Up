#include "libraidzero/controller/purePursuitController.hpp"
#include "libraidzero/pathing/purePursuitPath.hpp"
#include "libraidzero/util/mathUtil.hpp"

void PurePursuitController::setPath(
    const std::shared_ptr<PurePursuitPath> &ipath, QLength ilookahead
) {
    reset();
    path = ipath;
    currentLookahead = ilookahead;
}

PurePursuitPath::Point PurePursuitController::calculate(const Pose2d& ipose) {
    double targetVelocity = path->getClosestPoint(ipose).targetVelocity;
    Pose2d lookaheadPose = getLookaheadPoint(ipose);
    return {lookaheadPose, 0.0, 0.0, targetVelocity};
}

void PurePursuitController::reset() {
    path = nullptr;
    currentLookahead = 0_m;
    lastLookaheadIndex = 0;
    lastFractionalIndex = 0;
}

std::optional<double> PurePursuitController::getIntersection(
    const PurePursuitPath::Point& istart, const PurePursuitPath::Point& iend, 
    const Pose2d& ipose, QLength ilookahead
) {
    auto d = iend.pose.translation() - istart.pose.translation();
    auto f = istart.pose.translation() - ipose.translation();

    double a = (d * d).convert(meter);
    double b = 2 * (f * d).convert(meter);
    double c = (f * f).convert(meter) - 
        ilookahead.convert(meter) * ilookahead.convert(meter);

    double discriminant = b * b - 4 * a * c;
    if (discriminant >= 0) {
        discriminant = std::sqrt(discriminant);
        double t1 = (-b - discriminant) / (2 * a);
        double t2 = (-b + discriminant) / (2 * a);

        // t2 is always farther along the path
        if (t2 >= 0 && t2 <= 1) {
            return t2;
        } 
        if (t1 >= 0 && t1 <= 1) {
            return t1;
        }
    }
    return {};
}

Pose2d PurePursuitController::getLookaheadPoint(const Pose2d& ipose) {
    // Return the last point if it is within the radius
    if (PurePursuitPath::distance(ipose, path->points.back().pose) 
        <= currentLookahead.convert(meter)
    ) {
        return path->points.back().pose;
    }
    int pointCount = path->getPointCount();
    int lastIntersectIndex = 0;
    for (int i = lastLookaheadIndex; i < pointCount - 1; ++i) {
        auto fractionalIndex = PurePursuitController::getIntersection(
            path->points[i], path->points[i + 1], ipose, currentLookahead
        );
        if (fractionalIndex.has_value() && (i > lastLookaheadIndex || fractionalIndex.value() > lastFractionalIndex)) {
            lastLookaheadIndex = i;
            lastFractionalIndex = fractionalIndex.value();

            if (lastIntersectIndex > 0) {
                break;
            }
            lastIntersectIndex = i;
        }
        double d = PurePursuitPath::distance(
            path->points[i].pose, path->points[lastIntersectIndex].pose
        );
        if (lastIntersectIndex > 0 && d >= 2 * currentLookahead.convert(meter)) {
            break;
        }
    }
    const auto& start = path->points[lastLookaheadIndex].pose;
    const auto& end = path->points[lastLookaheadIndex + 1].pose;

    // Linearly interpolate between the two points
    const auto translation = lerp(start.translation(), end.translation(), lastFractionalIndex);
    const auto rotation = lerp(start.rotation(), end.rotation(), lastFractionalIndex);
    return {translation, rotation};
}