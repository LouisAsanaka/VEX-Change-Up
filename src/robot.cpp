#include "main.h"
#include "robot.hpp"

namespace robot {

	std::unique_ptr<Drive> drive;
	std::unique_ptr<Intake> intake;
	std::unique_ptr<Conveyor> conveyor;

	void init() {
		drive = std::make_unique<Drive>();
		intake = std::make_unique<Intake>();
		conveyor = std::make_unique<Conveyor>();
		conveyor->startTask("ConveyorTask");
		if (NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
			conveyor->notifyWhenDeletingRaw(pros::c::task_get_current());
		}
		pros::delay(200);
		conveyor->enableSensors();
	}

}
