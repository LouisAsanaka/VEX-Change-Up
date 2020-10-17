#include "main.h"
#include "opcontrol.hpp"
#include "robot.hpp"
#include "constants.hpp"
#include "robot/conveyor.hpp"
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
		bool forwardAdjust = master.getDigital(ControllerDigital::X);
		bool backwardAdjust = master.getDigital(ControllerDigital::B);
		if (forwardAdjust || backwardAdjust) {
			robot::drive::model->xArcade(
				0.0, forwardAdjust ? 0.3 : -0.3, 0.0, 0.05
			);
		} else {
			robot::drive::fieldOrientedControl(
				master.getAnalog(ControllerAnalog::leftX),
				master.getAnalog(ControllerAnalog::leftY),
				master.getAnalog(ControllerAnalog::rightX),
				0.05
			);
		}

		// std::cout << robot::drive::controller->getState().str() << std::endl;
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
		bool topUp = master.getDigital(ControllerDigital::L1);
		bool bottomUp = master.getDigital(ControllerDigital::L2);
		if (topUp) {
			robot::conveyor::moveUp(1.0, robot::conveyor::Position::Top);
			robot::conveyor::moveUp(1.0, robot::conveyor::Position::Bottom);
		} else if (bottomUp) {
			robot::conveyor::moveDown(1.0, robot::conveyor::Position::Top);
			robot::conveyor::moveDown(1.0, robot::conveyor::Position::Bottom);
		} else {
			robot::conveyor::stop();
		}
		// bool topDown = master.getDigital(ControllerDigital::left);
		// bool bottomDown = master.getDigital(ControllerDigital::down);
		// if (topUp || bottomUp || topDown || bottomDown) {
		// 	if (topUp) {
		// 		robot::conveyor::moveUp(1.0, robot::conveyor::Position::Top);
		// 	} else if (topDown) {
		// 		robot::conveyor::moveDown(1.0, robot::conveyor::Position::Top);
		// 	}
		// 	if (bottomUp) {
		// 		robot::conveyor::moveUp(1.0, robot::conveyor::Position::Bottom);
		// 	} else if (bottomDown) {
		// 		robot::conveyor::moveDown(1.0, robot::conveyor::Position::Bottom);
		// 	}
		// } else {
		// 	robot::conveyor::stop();
		// }
		//std::cout << lineTracker.get_value() << std::endl;
		pros::delay(20);
	}
}
