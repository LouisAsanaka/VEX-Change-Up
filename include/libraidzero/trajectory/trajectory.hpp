#pragma once

#include "main.h"
#include "libraidzero/geometry/pose2d.hpp"
#include <vector>

#define lerp(fromV, toV, iFrac) fromV + (toV - fromV) * iFrac

class Trajectory {
public:
    struct State {
        // Time (s) elapsed since the start
        double t = 0.0;
        // Velocity (m/s) at this point
        double vel = 0.0;
        // Acceleration (m/s^2) at this point
        double accel = 0.0;
        // Angular velocity (rad/s) at this point
        double angularVel = 0.0;
        // Pose at this point
        Pose2d pose;

        State(double t, double vel, double accel, double angularVel, const Pose2d& pose)
            : t{t}, vel{vel}, accel{accel}, angularVel{angularVel}, pose{pose} {}
    };

    Trajectory() = default;
    explicit Trajectory(std::vector<State> list);

    const std::vector<State>& getStates() const { return states; }
    double getTotalTime() const { return totalTime; }
    int getLength() const { return states.size(); }

    State sample(double t) const;

    using SegmentPtr = std::unique_ptr<Segment, void (*)(void *)>;

    static std::vector<Trajectory::State> segmentToStates(const SegmentPtr& traj, int length);
private:
    std::vector<State> states;
    double totalTime = 0.0;

    State interpolate(const State& start, const State& end, double i) const;
};