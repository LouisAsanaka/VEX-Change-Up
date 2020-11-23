#include "libraidzero/util/plotter.hpp"
#include "main.h"
#include "opcontrol.hpp"
#include "robot.hpp"
#include "constants.hpp"
#include "robot/conveyor.hpp"
#include "robot/drive.hpp"
#include <sstream>
#include <string>

void opcontrol() {
	Controller master {ControllerId::master};

	bool xDriveRegularOverride = false;
	bool overrideLatch = false;

	master.setText(0, 0, "Mode: field");

	bool warning15Seconds = false;

	robot::drive::resetEncoders();
	robot::drive::controller->setState({0_m, 0_m, 0_deg});

	int startTime = pros::millis();
	while (true) {
		// Dual joystick arcade
        // Source: https://www.vexforum.com/t/robot-c-arcade-drive/43348/2
		/*double forwardSpeed = master.getAnalog(ControllerAnalog::leftY) * DRIVE_SPEED;
		double yaw = master.getAnalog(ControllerAnalog::rightX) * DRIVE_SPEED;
		robot::drive::model->arcade(
			forwardSpeed, yaw, CONTROLLER_DEADBAND);*/
		//auto pose = Pose2d::fromOdomState(robot::drive::controller->getState());
		//std::cout << pose.toString() << std::endl;
		
		if (
			master.getDigital(ControllerDigital::left) && 
			master.getDigital(ControllerDigital::up)
		) {
			if (!overrideLatch) {
				xDriveRegularOverride = !xDriveRegularOverride;
				overrideLatch = true;
				if (xDriveRegularOverride) {
					master.rumble(".-");
					master.setText(0, 0, "Mode: robot");
				} else {
					master.rumble("-.");
					master.setText(0, 0, "Mode: field");
				}
			}
		} else {
			overrideLatch = false;
		}

		bool forwardAdjust = master.getDigital(ControllerDigital::X);
		bool backwardAdjust = master.getDigital(ControllerDigital::B);
		if (forwardAdjust || backwardAdjust) {
			robot::drive::model->xArcade(
				0.0, forwardAdjust ? 0.3 : -0.3, 0.0, 0.05
			);
		} else if (xDriveRegularOverride) {
			robot::drive::model->xArcade(
				master.getAnalog(ControllerAnalog::leftX),
				master.getAnalog(ControllerAnalog::leftY),
				master.getAnalog(ControllerAnalog::rightX),
				0.05
			);
		} else {
			robot::drive::fieldOrientedControl(
				master.getAnalog(ControllerAnalog::leftX),
				master.getAnalog(ControllerAnalog::leftY),
				master.getAnalog(ControllerAnalog::rightX),
				0.05
			);
		}

		//std::cout << robot::drive::controller->getState().str() << std::endl;
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
			robot::conveyor::stopAll();
		}
		if (!warning15Seconds && ((pros::millis() - startTime) > TIME_TILL_15_SECONDS)) {
			master.rumble("...");
			warning15Seconds = true;
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
