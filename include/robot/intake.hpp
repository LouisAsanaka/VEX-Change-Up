#pragma once

#include "main.h"
#include "libraidzero/api.hpp"

namespace robot::intake {

	extern std::unique_ptr<MotorController> controller;

	void init();

	void spinIn(double);
	void spinOut(double);
	void stop();
}
