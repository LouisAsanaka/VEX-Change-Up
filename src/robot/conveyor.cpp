#include "robot/conveyor.hpp"
#include "main.h"
#include "libraidzero/api.hpp"

namespace robot::conveyor {

	std::unique_ptr<MotorController> controller;

	void init() {
		MotorGroup motors {{-9, 10}};
		motors.setGearing(AbstractMotor::gearset::green);
		motors.setBrakeMode(AbstractMotor::brakeMode::brake);

		controller = std::make_unique<MotorController>(motors);
		controller->tarePosition();
	}

    void moveUp(double voltageScale) {
		controller->moveVoltage(voltageScale);
	}

	void moveDown(double voltageScale) {
		controller->moveVoltage(-voltageScale);
	}

	void stop() {
		controller->moveVoltage(0);
	}
}
