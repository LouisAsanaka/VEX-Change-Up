#pragma once

#include "main.h"
#include "libraidzero/api.hpp"

namespace robot::conveyor {

	enum class Position {
		Top, Bottom
	};

	extern std::unique_ptr<MotorController> topController;
	extern std::unique_ptr<MotorController> bottomController;
	extern CrossplatformThread* task;

	extern pros::ADIAnalogIn lineTracker;
	extern int average;

	extern std::atomic_bool checkingForBalls;
	extern std::atomic_int targetBallsPassed;

	void init();

	void moveUp(double, Position);
	void moveDown(double, Position);
	void moveBoth(double);
	void stopAll();
	void stop(Position);

	void startCountingBalls();
	void waitUntilPassed(int);

	void startThread();
	void trampoline(void *);
}
