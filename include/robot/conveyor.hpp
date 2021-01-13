#pragma once

#include "libraidzero/util/taskWrapper.hpp"
#include "main.h"
#include "libraidzero/api.hpp"

namespace robot {

class Conveyor : public TaskWrapper {
public:
	enum class Position {
		Top, Bottom, None
	};
	enum class ControlMode {
		Voltage, ScoreCount, StoreBall, WaitUntilEmpty
	};

	std::unique_ptr<MotorController> topController;
	std::unique_ptr<MotorController> bottomController;

	pros::ADIAnalogIn bottomLineTracker;
	int bottomAverage{0};
	pros::ADIAnalogIn topLineTracker;
	int topAverage{0};

	std::atomic_bool isIndexing{false};
	std::atomic<ControlMode> controlMode{ControlMode::Voltage};
	std::atomic_int targetBallsPassed{0};
	std::atomic<Position> storedPosition{Position::None};

	Conveyor();

	void moveUp(double, Position);
	void moveDown(double, Position);
	void moveBoth(double);
	void stopAll();
	void stop(Position);

	void startIndexing(ControlMode);
	void waitUntilPassed(int);
	void waitUntilStored(Position);
	void waitUntilEmpty(Position);

	void calibrate();

	void loop() override;
};

} // namespace robot
