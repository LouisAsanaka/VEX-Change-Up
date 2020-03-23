#include "builder/asyncRamsetePathControllerBuilder.hpp"
#include "main.h"
#include "controller/asyncRamsetePathController.hpp"

AsyncRamsetePathControllerBuilder::AsyncRamsetePathControllerBuilder(
    const std::shared_ptr<Logger> &ilogger)
    : logger(ilogger) {
}

AsyncRamsetePathControllerBuilder &AsyncRamsetePathControllerBuilder::withOutput(
    const std::shared_ptr<OdomChassisController> &icontroller) {
    hasChassisController = true;
    chassisController = icontroller;
    return *this;
}

AsyncRamsetePathControllerBuilder &
AsyncRamsetePathControllerBuilder::withLimits(const PathfinderLimits &ilimits) {
    hasLimits = true;
    limits = ilimits;
    return *this;
}

AsyncRamsetePathControllerBuilder &
AsyncRamsetePathControllerBuilder::withConstants(const RamseteConstants &iconstants) {
    constants = iconstants;
    return *this;
}

AsyncRamsetePathControllerBuilder &
AsyncRamsetePathControllerBuilder::withTimeUtilFactory(const TimeUtilFactory &itimeUtilFactory) {
    timeUtilFactory = itimeUtilFactory;
    return *this;
}

AsyncRamsetePathControllerBuilder &
AsyncRamsetePathControllerBuilder::withLogger(const std::shared_ptr<Logger> &ilogger) {
    controllerLogger = ilogger;
    return *this;
}

AsyncRamsetePathControllerBuilder &AsyncRamsetePathControllerBuilder::parentedToCurrentTask() {
    isParentedToCurrentTask = true;
    return *this;
}

AsyncRamsetePathControllerBuilder &
AsyncRamsetePathControllerBuilder::notParentedToCurrentTask() {
    isParentedToCurrentTask = false;
    return *this;
}

std::shared_ptr<AsyncRamsetePathController>
AsyncRamsetePathControllerBuilder::buildRamsetePathController() {
    if (!hasChassisController) {
        std::string msg("AsyncRamsetePathControllerBuilder: No chassis controller given.");
        LOG_ERROR(msg);
        throw std::runtime_error(msg);
    }

    if (!hasLimits) {
        std::string msg("AsyncRamsetePathControllerBuilder: No limits given.");
        LOG_ERROR(msg);
        throw std::runtime_error(msg);
    }

    auto out = std::make_shared<AsyncRamsetePathController>(
        timeUtilFactory.create(), limits, chassisController, constants);
    out->startThread();

    if (isParentedToCurrentTask && NOT_INITIALIZE_TASK && NOT_COMP_INITIALIZE_TASK) {
        out->getThread()->notifyWhenDeletingRaw(pros::c::task_get_current());
    }

    return out;
}