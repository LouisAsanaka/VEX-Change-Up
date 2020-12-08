#pragma once

#include "main.h"
#include "libraidzero/api.hpp"

namespace robot {

class Intake {
public:
	std::unique_ptr<MotorController> controller;

	Intake();

	void spinIn(double) const;
	void spinOut(double) const;
	void stop() const;
};

} // namespace robot
