#include "main.h"
#include "opcontrol.hpp"
#include "robot.hpp"
#include "constants.hpp"
#include "robot/drive.hpp"

void opcontrol() {
	Controller master {ControllerId::master};

	robot::drive::resetEncoders();
	robot::drive::controller->setState({0_m, 0_m, 0_deg});
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
		std::cout << robot::drive::controller->getState().str() << std::endl;
		if (master.getDigital(ControllerDigital::A)) {
			robot::drive::model->resetGyro();
		}
		if (master.getDigital(ControllerDigital::R1)) {
			robot::intake::spinIn(1.0);
		} else if (master.getDigital(ControllerDigital::R2)) {
			robot::intake::spinOut(1.0);
		} else {
			robot::intake::stop();
		}
		if (master.getDigital(ControllerDigital::L1)) {
			robot::conveyor::moveUp(1.0);
		} else if (master.getDigital(ControllerDigital::L2)) {
			robot::conveyor::moveDown(1.0);
		} else {
			robot::conveyor::stop();
		}
		pros::delay(20);
	}
}
