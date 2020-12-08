#include "libraidzero/controller/motorController.hpp"

#include "okapi/api.hpp"

MotorController::MotorController(const MotorGroup& motorRef) :
        motor {std::make_shared<MotorGroup>(motorRef)},
        maxVelocity {toUnderlyingType(motor->getGearing())},
        posController {
            std::static_pointer_cast<AsyncPosIntegratedController>(
                AsyncPosControllerBuilder()
                    .withMotor(motor)
                    .withMaxVelocity(maxVelocity)
                    .withGearset(motor->getGearing())
                    .build()
            )
        }
{
    posController->flipDisable(true);
}

std::shared_ptr<MotorGroup> MotorController::getMotor() const {
    return motor;
}

double MotorController::getPosition() {
    return motor->getPosition();
}

double MotorController::getTarget() {
    return posController->getTarget();
}

void MotorController::moveVoltage(double voltageScale) {
    if (!posController->isDisabled()) {
        posController->flipDisable(true);
    }
    motor->moveVoltage(MAX_VOLTAGE * voltageScale);
}

void MotorController::movePositionAsync(double position, double velocityScale) {
    tarePosition();
    posController->setMaxVelocity(maxVelocity * velocityScale);
    posController->setTarget(position);
    posController->flipDisable(false);
}

void MotorController::movePosition(double position, double velocityScale) {
    movePositionAsync(position, velocityScale);
    waitUntilSettled();
    posController->stop();
}

void MotorController::reset() {
    posController->reset();
}

void MotorController::tarePosition() {
    posController->tarePosition();
}

void MotorController::waitUntilSettled() {
    posController->waitUntilSettled();
}
