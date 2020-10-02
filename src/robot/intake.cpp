#include "robot/intake.hpp"
#include "main.h"
#include "libraidzero/api.hpp"

namespace robot::intake {

	std::unique_ptr<MotorController> controller;

	void init() {
		MotorGroup motors {{-6, 8}};
		motors.setGearing(AbstractMotor::gearset::green);
		motors.setBrakeMode(AbstractMotor::brakeMode::brake);

		controller = std::make_unique<MotorController>(motors);
		controller->tarePosition();
	}

	void spinIn(double voltageScale) {
		controller->moveVoltage(-voltageScale);
	}

	void spinOut(double voltageScale) {
		controller->moveVoltage(voltageScale);
	}

	void stop() {
		controller->moveVoltage(0);
	}
}
