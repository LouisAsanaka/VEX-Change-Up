#pragma once

#include "main.h"
#include "libraidzero/api.hpp"

namespace robot::conveyor {

	extern std::unique_ptr<MotorController> controller;

	void init();

	void moveUp(double);
	void moveDown(double);
	void stop();
}
