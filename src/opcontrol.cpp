#include "main.h"
#include "opcontrol.hpp"
#include "robot.hpp"
#include "constants.hpp"
#include "robot/drive.hpp"
#include "robot/intake.hpp"

void opcontrol() {
	Controller master {ControllerId::master};

	while (true) {
		// Dual joystick arcade
        // Source: https://www.vexforum.com/t/robot-c-arcade-drive/43348/2
		/*double forwardSpeed = master.getAnalog(ControllerAnalog::leftY) * DRIVE_SPEED;
		double yaw = master.getAnalog(ControllerAnalog::rightX) * DRIVE_SPEED;
		robot::drive::model->arcade(
			forwardSpeed, yaw, CONTROLLER_DEADBAND);*/
		robot::drive::fieldOrientedControl(
            master.getAnalog(ControllerAnalog::leftX),
            master.getAnalog(ControllerAnalog::leftY),
            master.getAnalog(ControllerAnalog::rightX),
            0.05
        );
		if (master.getDigital(ControllerDigital::R1)) {
			robot::intake::spinIn(1.0);
		} else if (master.getDigital(ControllerDigital::R2)) {
			robot::intake::spinOut(1.0);
		} else {
			robot::intake::stop();
		}
		pros::delay(20);
	}
}
