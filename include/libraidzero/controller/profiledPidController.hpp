#pragma once

#include "libraidzero/planner/trapezoidProfile.hpp"
#include "okapi/api/control/iterative/iterativePosPidController.hpp"
#include "okapi/api/units/QTime.hpp"

using namespace okapi;

class ProfiledPIDController {
public:
    using Gains = IterativePosPIDController::Gains;
    using Constraints = planner::TrapezoidProfile::Constraints;
    using State = planner::TrapezoidProfile::State;

    ProfiledPIDController(const Gains& igains, const TimeUtil &itimeUtil, 
        const Constraints& iconstrains, QTime isampleTime = 10_ms);

    void setGains(const Gains& igains);
    void setGoal(const State& igoal);
    void setGoal(double itarget);
    State getGoal() const;
    bool atGoal() const;
    void setConstraints(const Constraints& iconstraints);
    State getSetpoint() const;
    bool atSetpoint() const;

    double getError() const;
    double step(double inewReading);
    double step(double inewReading, double igoal);
    double step(double inewReading, double igoal, const Constraints& iconstraints);

    void reset(const State& istate);
    void reset(double iposition, double ivelocity = 0.0);
private:
    std::unique_ptr<IterativePosPIDController> pidController;
    State goal;
    State setpoint;
    Constraints constraints;
};