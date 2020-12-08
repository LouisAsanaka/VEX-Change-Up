#pragma once

#include "libraidzero/util/taskWrapper.hpp"
#include "main.h"
#include "libraidzero/api.hpp"

namespace robot {

class Conveyor : public TaskWrapper {
public:
	enum class Position {
		Top, Bottom
	};

	std::unique_ptr<MotorController> topController;
	std::unique_ptr<MotorController> bottomController;

	pros::ADIAnalogIn lineTracker;
	int average;

	std::atomic_bool checkingForBalls;
	std::atomic_int targetBallsPassed;

	Conveyor();

	void moveUp(double, Position);
	void moveDown(double, Position);
	void moveBoth(double);
	void stopAll();
	void stop(Position);

	void startCountingBalls();
	void waitUntilPassed(int);

	void loop() override;
};

} // namespace robot
