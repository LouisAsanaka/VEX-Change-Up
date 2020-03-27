#include "libraidzero/controller/pidController.hpp"
#include "okapi/api/units/QTime.hpp"
#include <algorithm>
#include <cmath>

PIDController::PIDController(double kP, double kI, double kD) : 
    kP{kP}, kI{kI}, kD{kD} 
{

}

void PIDController::setGains(double p, double i, double d) {
    kP = p;
    kI = i;
    kD = d;
}

void PIDController::setOutputLimits(double min, double max) {
    minOutput = min;
    maxOutput = max;
}

void PIDController::setSetpoint(double sp) {
    setpoint = sp;
}

double PIDController::getSetpoint() {
    return setpoint;
}

void PIDController::setTolerance(double positionTolerance, double velocityTolerance) {
    posTol = positionTolerance;
    velTol = velocityTolerance;
}

bool PIDController::atSetpoint() {
    return std::abs(positionError) < posTol && std::abs(velocityError) < velTol; 
}

double PIDController::calculate(double measurement, double sp) {
    setSetpoint(sp);
    return calculate(measurement);
}

double PIDController::calculate(double measurement) {
    previousError = positionError;

    positionError = setpoint - measurement;
    velocityError = (positionError - previousError) / timer->getDt().convert(okapi::second);

    totalError += positionError;
    totalError = std::clamp(totalError, minOutput, maxOutput);

    return std::clamp(kP * positionError + kI * totalError + kD * velocityError, minOutput, maxOutput);
}

double PIDController::getPositionError() {
    return positionError;
}

double PIDController::getVelocityError() {
    return velocityError;
}

void PIDController::reset() {
    previousError = 0.0;
    totalError = 0.0;
}
