#include "libraidzero/planner/trapezoidProfile.hpp"
#include <cmath>
#include <algorithm>

planner::TrapezoidProfile::TrapezoidProfile(
    const Constraints& iconstraints, const State& igoal, const State& iinitial
) : direction{shouldFlipAcceleration(iinitial, igoal) ? -1 : 1},
    constraints{iconstraints},
    initial{direct(iinitial)},
    goal{direct(igoal)}
{
    if (initial.velocity > constraints.maxVelocity) {
        initial.velocity = constraints.maxVelocity;
    }

    // Deal with a possibly truncated motion profile (with nonzero initial or
    // final velocity) by calculating the parameters as if the profile began and
    // ended at zero velocity
    double cutoffBegin = initial.velocity / constraints.maxAcceleration;
    double cutoffDistBegin = cutoffBegin * cutoffBegin 
        * constraints.maxAcceleration / 2.0;

    double cutoffEnd = goal.velocity / constraints.maxAcceleration;
    double cutoffDistEnd = cutoffEnd * cutoffEnd * constraints.maxAcceleration / 2.0;
    
    // Now we can calculate the parameters as if it was a full trapezoid instead
    // of a truncated one

    double fullTrapezoidDist = cutoffDistBegin + (goal.position - initial.position)
        + cutoffDistEnd;
    double accelerationTime = constraints.maxVelocity / constraints.maxAcceleration;

    double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime
        * constraints.maxAcceleration;

    // Handle the case where the profile never reaches full speed
    if (fullSpeedDist < 0) {
        accelerationTime = std::sqrt(fullTrapezoidDist / constraints.maxAcceleration);
        fullSpeedDist = 0;
    }

    endAcceleration = accelerationTime - cutoffBegin;
    endFullSpeed = endAcceleration + fullSpeedDist / constraints.maxVelocity;
    endDecceleration = endFullSpeed + accelerationTime - cutoffEnd;
}

planner::TrapezoidProfile::State planner::TrapezoidProfile::calculate(double t) {
    State result {initial.position, initial.velocity};

    if (t < endAcceleration) {
        result.velocity += t * constraints.maxAcceleration;
        result.position += (initial.velocity + t * constraints.maxAcceleration / 2.0) * t;
    } else if (t < endFullSpeed) {
        result.velocity = constraints.maxVelocity;
        result.position += (initial.velocity + endAcceleration * constraints.maxAcceleration
            / 2.0) * endAcceleration + constraints.maxVelocity * (t - endAcceleration);
    } else if (t <= endDecceleration) {
        result.velocity = goal.velocity + (endDecceleration - t) * constraints.maxAcceleration;
        double timeLeft = endDecceleration - t;
        result.position = goal.position - (goal.velocity + timeLeft
            * constraints.maxAcceleration / 2.0) * timeLeft;
    } else {
        result = goal;
    }

    return direct(result);
}

double planner::TrapezoidProfile::timeLeftUntil(double target) {
    double position = initial.position * direction;
    double velocity = initial.velocity * direction;

    double endAccel = endAccel * direction;
    double endFullSpeed = endFullSpeed * direction - endAccel;

    if (target < position) {
        endAccel = -endAccel;
        endFullSpeed = -endFullSpeed;
        velocity = -velocity;
    }

    endAccel = std::max(endAccel, 0.0);
    endFullSpeed = std::max(endFullSpeed, 0.0);
    double endDeccel = endDeccel - endAccel - endFullSpeed;
    endDeccel = std::max(endDeccel, 0.0);

    double acceleration = constraints.maxAcceleration;
    double decceleration = -constraints.maxAcceleration;

    double distToTarget = std::abs(target - position);
    if (distToTarget < 1e-6) {
        return 0;
    }

    double accelDist = velocity * endAccel + 0.5 * acceleration * endAccel * endAccel;

    double deccelVelocity;
    if (endAccel > 0) {
        deccelVelocity = std::sqrt(std::abs(velocity * velocity + 2 * acceleration * accelDist));
    } else {
        deccelVelocity = velocity;
    }

    double deccelDist = deccelVelocity * endDeccel + 0.5 * decceleration * endDeccel * endDeccel;

    deccelDist = std::max(deccelDist, 0.0);

    double fullSpeedDist = constraints.maxVelocity * endFullSpeed;

    if (accelDist > distToTarget) {
        accelDist = distToTarget;
        fullSpeedDist = 0;
        deccelDist = 0;
    } else if (accelDist + fullSpeedDist > distToTarget) {
        fullSpeedDist = distToTarget - accelDist;
        deccelDist = 0;
    } else {
        deccelDist = distToTarget - fullSpeedDist - accelDist;
    }

    double accelTime = (-velocity + std::sqrt(std::abs(velocity * velocity + 2 * acceleration
        * accelDist))) / acceleration;

    double deccelTime = (-deccelVelocity + std::sqrt(std::abs(deccelVelocity * deccelVelocity
        + 2 * decceleration * deccelDist))) / decceleration;

    double fullSpeedTime = fullSpeedDist / constraints.maxVelocity;

    return accelTime + fullSpeedTime + deccelTime;
}

double planner::TrapezoidProfile::totalTime() {
    return endDecceleration;
}

bool planner::TrapezoidProfile::isFinished(double t) {
    return t >= totalTime();
}

bool planner::TrapezoidProfile::shouldFlipAcceleration(
    const State& initial, const State& goal)
{
    return initial.position > goal.position;    
}

planner::TrapezoidProfile::State planner::TrapezoidProfile::direct(
    const planner::TrapezoidProfile::State& in) 
{
    planner::TrapezoidProfile::State result {in.position, in.velocity};
    result.position = result.position * direction;
    result.velocity = result.velocity * direction;
    return result;
}
