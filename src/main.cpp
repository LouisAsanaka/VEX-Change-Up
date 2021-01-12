#include "main.h"
#include "autonomous.hpp"
#include "gui.hpp"
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
    GUI::getInstance().initMenu();

    #if defined(RUN_WITHOUT_ROBOT) && RUN_WITHOUT_ROBOT
    #else
    robot::init();
    pros::delay(100);
    robot::conveyor->calibrate();
	#endif
}

void competition_initialize() {}

void disabled() {}
