#pragma once

#include "libraidzero/geometry/pose2d.hpp"
#include "libraidzero/util/mathUtil.hpp"

#include "okapi/api.hpp"

#include <vector>

using namespace okapi;

struct PurePursuitPath {
public:
    std::vector<Pose2d> points;

    PurePursuitPath(std::vector<Pose2d> iwaypoints, QLength ispacing);

    Pose2d getPoint(int iindex) const;

    void smoothen(double ia, double ib, double itolerance);
private:
    static std::vector<Pose2d> injectPoints(
        std::vector<Pose2d> iwaypoints, QLength spacing);
};
