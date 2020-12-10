#pragma once

#include "libraidzero/pathing/purePursuitPath.hpp"

#include "okapi/api.hpp"

class PurePursuitController {
public:

    struct Gains {
        double kP = 0.0;
        double kV = 0.0;
        double kA = 0.0;
    };

    void setPath(const std::shared_ptr<PurePursuitPath>& ipath, QLength ilookahead);
    PurePursuitPath::Point calculate(const Pose2d& ipose);

    void reset();
private:
    using Point = PurePursuitPath::Point;

    std::shared_ptr<PurePursuitPath> path {nullptr};
    QLength currentLookahead {0_m};
    int lastLookaheadIndex {0};
    double lastFractionalIndex {0.0};

    static std::optional<double> getIntersection(
        const Point& istart, const Point& iend, const Pose2d& ipose, 
        QLength ilookahead
    );

    Pose2d getLookaheadPoint(const Pose2d& ipose);

};