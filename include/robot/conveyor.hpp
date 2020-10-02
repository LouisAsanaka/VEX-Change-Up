#pragma once

#include "main.h"
#include "libraidzero/api.hpp"

namespace robot::conveyor {

	enum class Position {
		Top, Bottom
	};

	extern std::unique_ptr<MotorController> topController;
	extern std::unique_ptr<MotorController> bottomController;

	void init();

	void moveUp(double, Position);
	void moveDown(double, Position);
	void stop();
}
