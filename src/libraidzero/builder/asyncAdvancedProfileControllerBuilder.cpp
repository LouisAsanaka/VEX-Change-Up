#include "libraidzero/builder/asyncAdvancedProfileControllerBuilder.hpp"
#include "libraidzero/planner/profileStructs.hpp"
#include "main.h"
#include "libraidzero/controller/asyncAdvancedProfileController.hpp"

AsyncAdvancedProfileControllerBuilder::AsyncAdvancedProfileControllerBuilder(
    std::shared_ptr<Logger> ilogger)
    : logger(std::move(ilogger)) {
}

AsyncAdvancedProfileControllerBuilder &AsyncAdvancedProfileControllerBuilder::withOutput(
    const std::shared_ptr<ChassisController> &icontroller) {
    hasChassisController = true;
    chassisController = icontroller;
    return *this;
}

AsyncAdvancedProfileControllerBuilder &
AsyncAdvancedProfileControllerBuilder::withConfig(const planner::PlannerConfig &iconfig) {
    hasConfig = true;
    config = iconfig;
    return *this;
}

AsyncAdvancedProfileControllerBuilder &
AsyncAdvancedProfileControllerBuilder::withTimeUtilFactory(const TimeUtilFactory &itimeUtilFactory) {
    timeUtilFactory = itimeUtilFactory;
    return *this;
}

AsyncAdvancedProfileControllerBuilder &
AsyncAdvancedProfileControllerBuilder::withLogger(const std::shared_ptr<Logger> &ilogger) {
    controllerLogger = ilogger;
    return *this;
}

AsyncAdvancedProfileControllerBuilder &AsyncAdvancedProfileControllerBuilder::parentedToCurrentTask() {
    isParentedToCurrentTask = true;
    return *this;
}

AsyncAdvancedProfileControllerBuilder &
AsyncAdvancedProfileControllerBuilder::notParentedToCurrentTask() {
    isParentedToCurrentTask = false;
    return *this;
}

std::shared_ptr<AsyncAdvancedProfileController>
AsyncAdvancedProfileControllerBuilder::buildAdvancedProfileController() {
    if (!hasChassisController) {
        std::string msg("AsyncAdvancedProfileController: No chassis controller given.");
        LOG_ERROR(msg);
        throw std::runtime_error(msg);
    }

    if (!hasConfig) {
        std::string msg("AsyncAdvancedProfileController: No limits given.");
        LOG_ERROR(msg);
        throw std::runtime_error(msg);
    }

    auto out = std::make_shared<AsyncAdvancedProfileController>(
        timeUtilFactory.create(), config, chassisController->getModel(),
        chassisController->getChassisScales(), chassisController->getGearsetRatioPair(),
        logger
    );
    out->startThread();

    if (isParentedToCurrentTask && NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
        out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
    }

    return out;
}