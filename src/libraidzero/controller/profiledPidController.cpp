#include "libraidzero/controller/profiledPidController.hpp"
#include "libraidzero/planner/trapezoidProfile.hpp"

ProfiledPIDController::ProfiledPIDController(
    const Gains& igains, const TimeUtil &itimeUtil, 
    const Constraints& iconstrains, QTime isampleTime
) : pidController{std::make_unique<IterativePosPIDController>(igains, itimeUtil)},
    constraints{iconstrains}
{
    pidController->setSampleTime(isampleTime);
}

void ProfiledPIDController::setGains(const Gains& igains) {
    pidController->setGains(igains);
}

void ProfiledPIDController::setGoal(const State& igoal) {
    goal = igoal;
}

void ProfiledPIDController::setGoal(double itarget) {
    goal = {itarget, 0.0};
}

ProfiledPIDController::State ProfiledPIDController::getGoal() const {
    return goal;
}

bool ProfiledPIDController::atGoal() const {
    return atSetpoint() && goal == setpoint;
}

void ProfiledPIDController::setConstraints(const Constraints &iconstraints) {
    constraints = iconstraints;
}

ProfiledPIDController::State ProfiledPIDController::getSetpoint() const {
    return setpoint;
}

bool ProfiledPIDController::atSetpoint() const {
    return pidController->isSettled();
}

double ProfiledPIDController::getError() const {
    return pidController->getError();
}

double ProfiledPIDController::step(double inewReading) {
    planner::TrapezoidProfile profile{constraints, goal, setpoint};
    setpoint = profile.calculate(pidController->getSampleTime().convert(second));
    pidController->setTarget(setpoint.position);
    return pidController->step(inewReading);
}

double ProfiledPIDController::step(double inewReading, double igoal) {
    setGoal(igoal);
    return step(inewReading);
}

double ProfiledPIDController::step(double inewReading, double igoal, 
    const Constraints& iconstraints)
{
    setConstraints(iconstraints);
    return step(inewReading, igoal);
}

void ProfiledPIDController::reset(const State& istate) {
    pidController->reset();
    setpoint = istate;
}

void ProfiledPIDController::reset(double iposition, double ivelocity) {
    reset(State{iposition, ivelocity});
}
