#include "libraidzero/util/taskWrapper.hpp"

TaskWrapper::TaskWrapper(std::shared_ptr<okapi::Logger> ilogger) : 
    logger{std::move(ilogger)} 
{}

void TaskWrapper::loop() {
    std::string msg("TaskWrapper::loop: loop is not overridden");
    LOG_ERROR(msg);
    throw std::runtime_error(msg);
}

void TaskWrapper::startTask(const std::string& iname) {
    if (task) LOG_INFO("TaskWrapper::startTask: restarting task: " + iname);
    task = std::make_unique<CrossplatformThread>(trampoline, this, iname.c_str());
}

void TaskWrapper::killTask() {
    task = nullptr;
}

std::string TaskWrapper::getName() {
    return task->getName();
}

void TaskWrapper::notifyWhenDeletingRaw(const pros::task_t& itask) {
    task->notifyWhenDeletingRaw(itask);
}

void TaskWrapper::trampoline(void* icontext) {
    pros::delay(20);
    if (icontext != nullptr) {
        static_cast<TaskWrapper*>(icontext)->loop();
    }
}
