#pragma once

#include "main.h"
#include "libraidzero/api.hpp"

namespace robot {

class Conveyor : public TaskWrapper {
public:
	enum class RollerPosition {
		Top, Bottom
	};
	enum class BallPosition {
		Bottom = 0, Middle = 1, Top = 2
	};
	enum class ControlMode {
		Voltage, WaitUntilPassed, WaitUntilStored, WaitUntilEmpty
	};

	std::unique_ptr<MotorController> topController;
	std::unique_ptr<MotorController> bottomController;

	okapi::OpticalSensor midSensor;
	okapi::OpticalSensor topSensor;

	std::atomic_bool areSensorsReady{false};
	std::atomic_bool isIndexing{false};
	std::atomic<ControlMode> controlMode{ControlMode::Voltage};
	std::atomic<BallPosition> position;
	std::atomic_int ballsPassed{0};
	std::atomic_bool currentlyPassing{false};

	std::array<bool, 3> isBallPresent;

	Conveyor();

	void moveUp(double, RollerPosition);
	void moveDown(double, RollerPosition);
	void moveBoth(double);
	void stopAll();
	void stop(RollerPosition);

	void enableSensors();
	bool isBallIn(BallPosition);

	void startCountingPassed(BallPosition);
	void waitUntilPassed(int, int itimeout = 0);
	void waitUntilStored(BallPosition, int itimeout = 0);
	void waitUntilEmpty(BallPosition, int itimeout = 0);

	void loop() override;
private:
	void updateSensors();
};

} // namespace robot
