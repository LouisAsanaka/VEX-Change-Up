#include "robot/conveyor.hpp"
#include "main.h"
#include "libraidzero/api.hpp"
#include <atomic>

namespace robot {

Conveyor::Conveyor() 
	: TaskWrapper{}, 
	  bottomLineTracker{'B'}, topLineTracker{'A'}
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

	bottomAverage = 2700;
	topAverage = 2700;
}

void Conveyor::moveUp(double voltageScale, Position position) {
	if (position == Position::Top) {
		topController->moveVoltage(voltageScale);
	} else {
		bottomController->moveVoltage(voltageScale);
	}
}

void Conveyor::moveDown(double voltageScale, Position position) {
	if (position == Position::Top) {
		topController->moveVoltage(-voltageScale);
	} else {
		bottomController->moveVoltage(-voltageScale);
	}
}

void Conveyor::moveBoth(double voltageScale) {
	moveUp(voltageScale, Position::Top);
	moveUp(voltageScale, Position::Bottom);
}

void Conveyor::stopAll() {
	topController->moveVoltage(0);
	bottomController->moveVoltage(0);
}

void Conveyor::stop(Position position) {
	if (position == Position::Top) {
		topController->moveVoltage(0);
	} else {
		bottomController->moveVoltage(0);
	}
}

void Conveyor::startIndexing(ControlMode mode) {
	if (isIndexing.load(std::memory_order_acquire)) {
		return;
	}
	isIndexing.store(true, std::memory_order_release);
	controlMode.store(mode, std::memory_order_release);
	targetBallsPassed.store(99, std::memory_order_release);
}

void Conveyor::waitUntilPassed(int numberOfBalls) {
	if (!isIndexing.load(std::memory_order_acquire) || 
		controlMode.load(std::memory_order_acquire) != ControlMode::ScoreCount
	) {
		return;
	}
	targetBallsPassed.store(numberOfBalls, std::memory_order_release);
	while (isIndexing.load(std::memory_order_acquire)) {
		pros::delay(20);
	}
}

void Conveyor::waitUntilStored(Position position) {
	if (!isIndexing.load(std::memory_order_acquire) ||
		controlMode.load(std::memory_order_acquire) != ControlMode::StoreBall
	) {
		return;
	}
	storedPosition.store(position, std::memory_order_release);
	while (isIndexing.load(std::memory_order_acquire)) {
		pros::delay(20);
	}
}

void Conveyor::waitUntilEmpty(Position position) {
	if (!isIndexing.load(std::memory_order_acquire) ||
		controlMode.load(std::memory_order_acquire) != ControlMode::WaitUntilEmpty
	) {
		return;
	}
	storedPosition.store(position, std::memory_order_release);
	while (isIndexing.load(std::memory_order_acquire)) {
		pros::delay(20);
	}
}

void Conveyor::calibrate() {
	bottomAverage = bottomLineTracker.calibrate();
	topAverage = topLineTracker.calibrate();
}

void Conveyor::loop() {
	while (task->notifyTake(0) == 0U) {
		// std::cout << bottomLineTracker.get_value() << ": avg: " << bottomAverage << std::endl;
		if (isIndexing.load(std::memory_order_acquire)) {
			if (controlMode == ControlMode::ScoreCount) {
				int currentBallsPassed = 0;
				while (currentBallsPassed < targetBallsPassed.load(std::memory_order_release)) {
					//std::cout << average << " - " << lineTracker.get_value() << std::endl;
					if (bottomAverage - bottomLineTracker.get_value() > 400) {
						//std::cout << "passed" << std::endl;
						++currentBallsPassed;
					}
					pros::delay(5);
				}
				while (!(topAverage - topLineTracker.get_value() > 400)) {
					pros::delay(5);
				}
				stop(Position::Bottom);
				pros::delay(200);
				stop(Position::Top);
				isIndexing.store(false, std::memory_order_release);
				controlMode.store(ControlMode::Voltage, std::memory_order_release);
			} else if (controlMode == ControlMode::StoreBall) {
				while (storedPosition.load(std::memory_order_acquire) == Position::None) {
					pros::delay(20);
				}
				Position position = storedPosition.load(std::memory_order_acquire);
				if (position == Position::Top) {
					// Ignore currently stored ball
					while (bottomAverage - bottomLineTracker.get_value() > 400) {
						pros::delay(5);
					}
					while (!(topAverage - topLineTracker.get_value() > 400)) {
						pros::delay(5);
					}
				} else {
					// Ignore currently stored ball
					while (bottomAverage - bottomLineTracker.get_value() > 400) {
						pros::delay(5);
					}
					while (!(bottomAverage - bottomLineTracker.get_value() > 400)) {
						pros::delay(5);
					}
				}
				stop(position);
				isIndexing.store(false, std::memory_order_release);
				storedPosition.store(Position::None, std::memory_order_release);
				controlMode.store(ControlMode::Voltage, std::memory_order_release);
			} else if (controlMode == ControlMode::WaitUntilEmpty) {
				while (storedPosition.load(std::memory_order_acquire) == Position::None) {
					pros::delay(20);
				}
				Position position = storedPosition.load(std::memory_order_acquire);
				if (position == Position::Top) {
					while (topAverage - topLineTracker.get_value() > 700) {
						pros::delay(5);
					}
				} else {
					while (bottomAverage - bottomLineTracker.get_value() > 700) {
						pros::delay(5);
					}
				}
				stopAll();
				isIndexing.store(false, std::memory_order_release);
				storedPosition.store(Position::None, std::memory_order_release);
				controlMode.store(ControlMode::Voltage, std::memory_order_release);
			}
		}
		pros::delay(20);
	}
}
} // namespace robot
