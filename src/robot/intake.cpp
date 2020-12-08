#include "robot/intake.hpp"
#include "main.h"
#include "libraidzero/api.hpp"

namespace robot {

Intake::Intake() {
	MotorGroup motors {{-6, 8}};
	motors.setGearing(AbstractMotor::gearset::green);
	motors.setBrakeMode(AbstractMotor::brakeMode::brake);

	controller = std::make_unique<MotorController>(motors);
	controller->tarePosition();
}

void Intake::spinIn(double voltageScale) const {
	controller->moveVoltage(-voltageScale);
}

void Intake::spinOut(double voltageScale) const {
	controller->moveVoltage(voltageScale);
}

void Intake::stop() const {
	controller->moveVoltage(0);
}

} // namespace robot
