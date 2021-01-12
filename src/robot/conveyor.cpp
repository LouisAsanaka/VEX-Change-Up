#include "robot/conveyor.hpp"
#include "main.h"
#include "libraidzero/api.hpp"
#include <atomic>

namespace robot {

Conveyor::Conveyor() 
	: TaskWrapper{}, 
	  bottomLineTracker{'B'}, bottomAverage{0},
	  topLineTracker{'A'}, topAverage{0},
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

void Conveyor::calibrate() {
	bottomAverage = bottomLineTracker.calibrate();
	topAverage = topLineTracker.calibrate();
}

void Conveyor::loop() {
	while (task->notifyTake(0) == 0U) {
		//std::cout << lineTracker.get_value() << ": avg: " << average << std::endl;
		if (checkingForBalls.load(std::memory_order_acquire)) {
			int currentBallsPassed = 0;
			while (currentBallsPassed < targetBallsPassed.load(std::memory_order_release)) {
				//std::cout << average << " - " << lineTracker.get_value() << std::endl;
				if (bottomAverage - bottomLineTracker.get_value() > 400) {
					//std::cout << "passed" << std::endl;
					++currentBallsPassed;
				}
				pros::delay(10);
			}
			while (!(topAverage - topLineTracker.get_value() > 400)) {
				pros::delay(10);
			}
			stop(Position::Bottom);
			pros::delay(200);
			stop(Position::Top);
			checkingForBalls.store(false, std::memory_order_release);
		}
		pros::delay(100);
	}
}
} // namespace robot
