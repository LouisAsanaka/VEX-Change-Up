#include "main.h"
#include "robot.hpp"

namespace robot {

	void init() {
		robot::drive::init();
		robot::intake::init();
		robot::conveyor::init();
	}

}
