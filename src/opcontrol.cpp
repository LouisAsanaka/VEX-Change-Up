#include "main.h"
#include "opcontrol.hpp"
#include "robot.hpp"
#include "constants.hpp"

void opcontrol() {
	Controller master {ControllerId::master};

	while (true) {
		// Dual joystick arcade
        // Source: https://www.vexforum.com/t/robot-c-arcade-drive/43348/2
		/*double forwardSpeed = master.getAnalog(ControllerAnalog::leftY) * DRIVE_SPEED;
		double yaw = master.getAnalog(ControllerAnalog::rightX) * DRIVE_SPEED;
		robot::drive::model->arcade(
			forwardSpeed, yaw, CONTROLLER_DEADBAND);*/
		robot::drive::model->xArcade(
            master.getAnalog(ControllerAnalog::leftX),
            master.getAnalog(ControllerAnalog::leftY),
            master.getAnalog(ControllerAnalog::rightX),
            0.1
        );
		pros::delay(20);
	}
}
