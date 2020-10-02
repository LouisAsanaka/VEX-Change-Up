#include "robot/conveyor.hpp"
#include "main.h"
#include "libraidzero/api.hpp"

namespace robot::conveyor {

	std::unique_ptr<MotorController> topController;
	std::unique_ptr<MotorController> bottomController;

	void init() {
		MotorGroup topMotor {{10}};
		topMotor.setGearing(AbstractMotor::gearset::blue);
		topMotor.setBrakeMode(AbstractMotor::brakeMode::brake);

		topController = std::make_unique<MotorController>(topMotor);
		topController->tarePosition();

		MotorGroup bottomMotor {{-9}};
		bottomMotor.setGearing(AbstractMotor::gearset::blue);
		bottomMotor.setBrakeMode(AbstractMotor::brakeMode::brake);

		bottomController = std::make_unique<MotorController>(bottomMotor);
		bottomController->tarePosition();
	}

    void moveUp(double voltageScale, Position position) {
		if (position == Position::Top) {
			topController->moveVoltage(voltageScale);
		} else {
			bottomController->moveVoltage(voltageScale);
		}
	}

	void moveDown(double voltageScale, Position position) {
		if (position == Position::Top) {
			topController->moveVoltage(-voltageScale);
		} else {
			bottomController->moveVoltage(-voltageScale);
		}
	}

	void stop() {
		topController->moveVoltage(0);
		bottomController->moveVoltage(0);
	}
}
