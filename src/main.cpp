#include "main.h"
#include "autonomous.hpp"
#include "opcontrol.hpp"
#include "robot.hpp"

void initialize() {
	pros::lcd::initialize();
	robot::init();
}

void competition_initialize() {}

void disabled() {}
