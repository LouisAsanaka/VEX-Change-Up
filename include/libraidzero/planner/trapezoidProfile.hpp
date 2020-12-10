#pragma once

#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

#include <vector>

namespace planner {
    using namespace okapi::literals;

    class TrapezoidProfile {
    public:
        struct Constraints {
            double maxVelocity;
            double maxAcceleration;

            Constraints(double maxVelocity, double maxAcceleration) : 
                maxVelocity{maxVelocity}, maxAcceleration{maxAcceleration} {}
        };

        struct State {
            double position;
            double velocity;

            State(double position = 0.0, double velocity = 0.0) : 
                position{position}, velocity{velocity} {}

            bool operator==(const State& rhs) const {
                return position == rhs.position && velocity == rhs.velocity;
            }

            bool operator!=(const State& rhs) const { 
                return !(*this == rhs); 
            }
        };

        TrapezoidProfile(const Constraints& iconstraints, const State& igoal, 
            const State& iinitial = {0, 0});

        State calculate(double t);
        double timeLeftUntil(double target);
        double totalTime();
        bool isFinished(double t);
    private:
        int direction;
        Constraints constraints;
        State initial, goal;
        double endAcceleration, endFullSpeed, endDecceleration;

        static bool shouldFlipAcceleration(
            const State& initial, const State& goal);
        State direct(const State& in);
    };
};