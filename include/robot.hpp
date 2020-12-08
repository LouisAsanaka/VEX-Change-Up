#pragma once

#include "robot/drive.hpp"
#include "robot/intake.hpp"
#include "robot/conveyor.hpp"

#include <memory>

namespace robot {

	extern std::unique_ptr<Drive> drive;
	extern std::unique_ptr<Intake> intake;
	extern std::unique_ptr<Conveyor> conveyor;

	void init();
}
