#include "robot/conveyor.hpp"
#include "main.h"
#include "libraidzero/api.hpp"
#include <atomic>

namespace robot {

Conveyor::Conveyor() 
	: TaskWrapper{}, 
	  lineTracker{'B'}, average{0},
	  checkingForBalls{false}, targetBallsPassed{0} 
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

	average = lineTracker.calibrate();
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

void Conveyor::startCountingBalls() {
	if (checkingForBalls.load(std::memory_order_acquire)) {
		return;
	}
	checkingForBalls.store(true, std::memory_order_release);
	targetBallsPassed.store(99, std::memory_order_release);
}

void Conveyor::waitUntilPassed(int numberOfBalls) {
	if (!checkingForBalls.load(std::memory_order_acquire)) {
		return;
	}
	targetBallsPassed.store(numberOfBalls, std::memory_order_release);
	while (checkingForBalls.load(std::memory_order_acquire)) {
		pros::delay(100);
	}
}

void Conveyor::loop() {
	while (task->notifyTake(0) == 0U) {
		if (checkingForBalls.load(std::memory_order_acquire)) {
			int currentBallsPassed = 0;
			while (currentBallsPassed < targetBallsPassed.load(std::memory_order_release)) {
				//std::cout << average << " - " << lineTracker.get_value() << std::endl;
				if (average - lineTracker.get_value() > 400) {
					//std::cout << "passed" << std::endl;
					++currentBallsPassed;
				}
				pros::delay(50);
			}
			pros::delay(200);
			stop(Position::Bottom);
			pros::delay(400);
			stop(Position::Top);
			checkingForBalls.store(false, std::memory_order_release);
		}
		pros::delay(50);
	}
}
} // namespace robot
