#pragma once

#include "okapi/api.hpp"

#include <memory>

using namespace okapi;

class MotorController {
    static constexpr int MAX_VOLTAGE = 12000;
private:
    std::shared_ptr<MotorGroup> motor;
    int maxVelocity;
    std::shared_ptr<AsyncPosIntegratedController> posController;
public:
    /**
     * Generic motor controller that can control both voltage & position (PID).
     *
     * @param motor the motor to control
     */
    MotorController(const MotorGroup& motor);

    /**
     * Returns the motor.
     *
     * @return the motor
     */
    std::shared_ptr<MotorGroup> getMotor() const;

    /**
     * Returns the absolute position of the motor in its encoder units.
     *
     * @return position in encoder units
     */
    double getPosition();

    /**
     * Returns the target last set in the controller.
     *
     * @return last set target
     */
    double getTarget();


    /**
     * Sets the voltage of the motor.
     *
     * @param voltageScale percent voltage from -1 to 1
     */
    void moveVoltage(double voltageScale);

    /**
     * Sets the target for the controller.
     *
     * @param position      target position in motor units
     * @param velocityScale velocity scale from -1.0 to 1.0
     */
    void movePositionAsync(double position, double velocityScale = 1.0);

    /**
     * Sets the target for the controller.
     *
     * @param position      target position in motor units
     * @param velocityScale velocity scale from -1.0 to 1.0
     */
    void movePosition(double position, double velocityScale = 1.0);

    /**
     * Resets the controller's internal states.
     */
    void reset();

    /**
     * Resets the absolute position of the motor to zero.
     */
    void tarePosition();

    /**
     * Blocks the current task until the controller settles.
     */
    void waitUntilSettled();
};
