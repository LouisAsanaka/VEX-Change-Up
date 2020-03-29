#include "main.h"
#include "autonomous.hpp"
#include "opcontrol.hpp"
#include "robot.hpp"

void initialize() {
    Logger::setDefaultLogger(
        std::make_shared<Logger>(
            TimeUtilFactory::createDefault().getTimer(),
            "/ser/sout", // Output to the PROS terminal
            Logger::LogLevel::info
        )
    );
	pros::lcd::initialize();
	robot::init();
}

void competition_initialize() {}

void disabled() {}
