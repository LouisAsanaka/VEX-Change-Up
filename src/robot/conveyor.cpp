#include "robot/conveyor.hpp"
#include "main.h"
#include "constants.hpp"
#include "libraidzero/api.hpp"
#include <atomic>

namespace robot {

Conveyor::Conveyor() 
	: TaskWrapper{}, 
	  midSensor{16}, topSensor{5}
{
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

void Conveyor::moveUp(double voltageScale, RollerPosition position) {
	if (position == RollerPosition::Top) {
		topController->moveVoltage(voltageScale);
	} else {
		bottomController->moveVoltage(voltageScale);
	}
}

void Conveyor::moveDown(double voltageScale, RollerPosition position) {
	if (position == RollerPosition::Top) {
		topController->moveVoltage(-voltageScale);
	} else {
		bottomController->moveVoltage(-voltageScale);
	}
}

void Conveyor::moveBoth(double voltageScale) {
	moveUp(voltageScale, RollerPosition::Top);
	moveUp(voltageScale, RollerPosition::Bottom);
}

void Conveyor::stopAll() {
	topController->moveVoltage(0);
	bottomController->moveVoltage(0);
}

void Conveyor::stop(RollerPosition position) {
	if (position == RollerPosition::Top) {
		topController->moveVoltage(0);
	} else {
		bottomController->moveVoltage(0);
	}
}

void Conveyor::enableSensors() {
	areSensorsReady.store(true, std::memory_order_release);
}

bool Conveyor::isBallIn(BallPosition position) {
	return isBallPresent[static_cast<int>(position)];
}

void Conveyor::startCountingPassed(BallPosition iposition) {
	controlMode.store(ControlMode::WaitUntilPassed, std::memory_order_release);
	position.store(iposition, std::memory_order_release);
	ballsPassed.store(0, std::memory_order_release);
	currentlyPassing.store(false, std::memory_order_release);
}

void Conveyor::waitUntilPassed(int number, int itimeout) {
	if (itimeout == 0) {
        itimeout = ~itimeout;
    }
	
	int startTime = pros::millis();
	while (
		ballsPassed.load(std::memory_order_acquire) < number &&
		pros::millis() - startTime < itimeout
	) {
		pros::delay(5);
	}
	controlMode.store(ControlMode::Voltage, std::memory_order_release);
}

void Conveyor::waitUntilStored(BallPosition iposition, int itimeout) {
	if (itimeout == 0) {
        itimeout = ~itimeout;
    }

	controlMode.store(ControlMode::WaitUntilStored, std::memory_order_release);
	position.store(iposition, std::memory_order_release);

	int startTime = pros::millis();
	while (!isBallIn(iposition) && pros::millis() - startTime < itimeout) {
		pros::delay(5);
	}
	controlMode.store(ControlMode::Voltage, std::memory_order_release);
}

void Conveyor::waitUntilEmpty(BallPosition iposition, int itimeout) {
	if (itimeout == 0) {
        itimeout = ~itimeout;
    }

	controlMode.store(ControlMode::WaitUntilEmpty, std::memory_order_release);
	position.store(iposition, std::memory_order_release);

	int startTime = pros::millis();
	while (isBallIn(iposition) && pros::millis() - startTime < itimeout) {
		pros::delay(5);
	}
	controlMode.store(ControlMode::Voltage, std::memory_order_release);
}

void Conveyor::loop() {
	std::cout << "[Conveyor] Waiting for optical sensors..." << std::endl;
	while (!areSensorsReady.load(std::memory_order_acquire)) {
		pros::delay(20);
	}
	std::cout << "[Conveyor] Optical sensors are ready!" << std::endl;
	while (
		areSensorsReady.load(std::memory_order_acquire) && 
		task->notifyTake(0) == 0U
	) {
		updateSensors();

		auto mode = controlMode.load(std::memory_order_acquire);
		if (mode == ControlMode::Voltage) {
			continue;
		}
		if (mode == ControlMode::WaitUntilPassed) {
			bool passing = currentlyPassing.load(std::memory_order_acquire);
			if (isBallIn(position)) {
				if (!passing) {
					currentlyPassing.store(true, std::memory_order_release);
				}
			} else if (passing) {
				currentlyPassing.store(false, std::memory_order_release);
				ballsPassed.store(
					ballsPassed.load(std::memory_order_acquire) + 1, 
					std::memory_order_release
				);
			}
		}
		//std::cout << "Proximity values: " << midSensor.getProximity() << " | " << topSensor.getProximity() << std::endl;
		pros::delay(5);
	}
}

void Conveyor::updateSensors() {
	isBallPresent[static_cast<int>(BallPosition::Middle)] = 
		midSensor.getProximity() > CONVEYOR_OPTICAL_THRESHOLD;
	isBallPresent[static_cast<int>(BallPosition::Top)] = 
		topSensor.getProximity() > CONVEYOR_OPTICAL_THRESHOLD;
}
} // namespace robot
