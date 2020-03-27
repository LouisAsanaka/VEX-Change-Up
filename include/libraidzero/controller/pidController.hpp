#pragma once

#include "okapi/impl/util/timeUtilFactory.hpp"
#include "okapi/api/util/abstractTimer.hpp"
#include "okapi/api/units/QTime.hpp"
#include <limits>
#include <memory>

class PIDController {
public:
    PIDController(double kP, double kI, double kD);
    ~PIDController() = default;

    PIDController(const PIDController&) = default;
    PIDController& operator=(const PIDController&) = default;
    PIDController(PIDController&&) = default;
    PIDController& operator=(PIDController&&) = default;

    void setGains(double kP, double kI, double kD);
    void setOutputLimits(double min, double max);

    void setSetpoint(double sp);
    double getSetpoint();

    void setTolerance(double positionTolerance,
                      double velocityTolerance = std::numeric_limits<double>::infinity());
    bool atSetpoint();

    double calculate(double measurement, double sp);
    double calculate(double measurement);

    double getPositionError();
    double getVelocityError();

    void reset();
private:
    std::unique_ptr<okapi::AbstractTimer> timer = okapi::TimeUtilFactory().createDefault().getTimer();

    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;

    double minOutput = -1.0;
    double maxOutput = 1.0;

    double posTol = 0.1;
    double velTol = std::numeric_limits<double>::infinity();

    double setpoint = 0.0;

    double positionError = 0;
    double velocityError = 0;

    double previousError = 0.0;
    double totalError = 0.0;
};