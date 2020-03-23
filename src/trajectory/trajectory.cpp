#include "trajectory/trajectory.hpp"
#include "main.h"
#include "util/miscUtil.hpp"
#include <vector>

Trajectory::Trajectory(const std::vector<State>& list) : states(list) {
    totalTime = states.back().t;
}

Trajectory::State Trajectory::interpolate(const State& start, const State& end, double i) const {
    // Find the new [t] value.
    const double newT = lerp(start.t, end.t, i);

    // Find the delta time between the current state and the interpolated state.
    const double deltaT = newT - start.t;

    // If delta time is negative, flip the order of interpolation.
    if (deltaT < 0) {
        return interpolate(end, start, 1.0 - i);
    }

    // Check whether the robot is reversing at this stage.
    const auto reversing =
        start.vel < 0 ||
        (std::abs(start.vel) < 1e-9 && start.accel < 0);

    // Calculate the new velocity.
    // v = v_0 + at
    const double newV = start.vel + (start.accel * deltaT);

    // Calculate the change in position.
    // delta_s = v_0 t + 0.5 at^2
    const double newS =
        (start.vel * deltaT + 0.5 * start.accel * deltaT * deltaT) *
        (reversing ? -1.0 : 1.0);

    // Return the new state. To find the new position for the new state, we need
    // to interpolate between the two endpoint poses. The fraction for
    // interpolation is the change in position (delta s) divided by the total
    // distance between the two endpoints.
    const double interpolationFrac =
        newS / end.pose.translation().distance(
            start.pose.translation()).convert(meter);

    return {
        newT, newV, start.accel, lerp(start.angularVel, end.angularVel, i),
        lerp(start.pose, end.pose, interpolationFrac)
    };
}

Trajectory::State Trajectory::sample(double t) const {
    if (t <= states.front().t) {
        return states.front();
    };
    if (t >= totalTime) {
        return states.back();
    }

    // To get the element that we want, we will use a binary search algorithm
    // instead of iterating over a for-loop. A binary search is O(std::log(n))
    // whereas searching using a loop is O(n).

    // This starts at 1 because we use the previous state later on for
    // interpolation.
    int low = 1;
    int high = states.size() - 1;

    while (low != high) {
        int mid = (low + high) / 2;
        if (states[mid].t < t) {
            // This index and everything under it are less than the requested
            // timestamp. Therefore, we can discard them.
            low = mid + 1;
        } else {
            // t is at least as large as the element at this index. This means that
            // anything after it cannot be what we are looking for.
            high = mid;
        }
    }

    // High and Low should be the same.

    // The sample's timestamp is now greater than or equal to the requested
    // timestamp. If it is greater, we need to interpolate between the
    // previous state and the current state to get the exact state that we
    // want.
    const State sample = states[low];
    const State prevSample = states[low - 1];

    // If the difference in states is negligible, then we are spot on!
    if (std::abs(sample.t - prevSample.t) < 1e-9) {
        return sample;
    }

    // Interpolate between the two states for the state that we want.
    return interpolate(prevSample, sample,
        (t - prevSample.t) / (sample.t - prevSample.t));
}

std::vector<Trajectory::State> Trajectory::segmentToStates(SegmentPtr& traj, int length) {
    std::vector<Trajectory::State> states;
    states.reserve(length);

    // Accumulate time
    double t = 0.0; 
    for (int i = 0; i < length; ++i) {
        auto segment = traj.get()[i];
        double angularVel;
        if (i == length - 1) {
            angularVel = 0.0;
        } else {
            auto& nextSegment = traj.get()[i + 1];
            angularVel = (constrainAngle(nextSegment.heading) - constrainAngle(segment.heading))
                / nextSegment.dt;
        }
        states.emplace_back(t, segment.velocity, segment.acceleration, angularVel,
            Pose2d{
                segment.x * meter, segment.y * meter,
                Rotation2d{constrainAngle(segment.heading) * radian}
            }
        );
        // std::cout << states[i].pose.toString() << std::endl;
        t += segment.dt;
    }
    std::cout << "-----------------------" << std::endl;
    std::cout << "Length: " << length << " waypoints" << std::endl;
    std::cout << "Duration: " << t << " s" << std::endl;
    return states;
}
