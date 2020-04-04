#pragma once

#include "main.h"
#include "libraidzero/controller/asyncAdvancedProfileController.hpp"
#include "libraidzero/planner/profileStructs.hpp"

class AsyncAdvancedProfileControllerBuilder {
public:
    /**
     * A builder that creates async Ramsete path controllers. Use this to build an
     * AsyncRamsetePathController.
     *
     * @param ilogger The logger this instance will log to.
     */
    explicit AsyncAdvancedProfileControllerBuilder(
        std::shared_ptr<Logger> ilogger = Logger::getDefaultLogger());

    /**
     * Sets the output. This must be used with buildRamsetePathController().
     *
     * @param icontroller The chassis controller to use.
     * @return An ongoing builder.
     */
    AsyncAdvancedProfileControllerBuilder &
    withOutput(const std::shared_ptr<ChassisController> &icontroller);

    /**
     * Sets the config.
     *
     * @param ilimits The config.
     * @return An ongoing builder.
     */
    AsyncAdvancedProfileControllerBuilder &withConfig(const planner::PlannerConfig &ilimits);

    /**
     * Sets the TimeUtilFactory used when building the controller. The default is the static
     * TimeUtilFactory.
     *
     * @param itimeUtilFactory The TimeUtilFactory.
     * @return An ongoing builder.
     */
    AsyncAdvancedProfileControllerBuilder &withTimeUtilFactory(const TimeUtilFactory &itimeUtilFactory);

    /**
     * Sets the logger.
     *
     * @param ilogger The logger.
     * @return An ongoing builder.
     */
    AsyncAdvancedProfileControllerBuilder &withLogger(const std::shared_ptr<Logger> &ilogger);

    /**
     * Parents the internal tasks started by this builder to the current task, meaning they will be
     * deleted once the current task is deleted. The `initialize` and `competition_initialize` tasks
     * are never parented to. This is the default behavior.
     *
     * Read more about this in the [builders and tasks tutorial]
     * (docs/tutorials/concepts/builders-and-tasks.md).
     *
     * @return An ongoing builder.
     */
    AsyncAdvancedProfileControllerBuilder &parentedToCurrentTask();

    /**
     * Prevents parenting the internal tasks started by this builder to the current task, meaning they
     * will not be deleted once the current task is deleted. This can cause runaway tasks, but is
     * sometimes the desired behavior (e.x., if you want to use this builder once in `autonomous` and
     * then again in `opcontrol`).
     *
     * Read more about this in the [builders and tasks tutorial]
     * (docs/tutorials/concepts/builders-and-tasks.md).
     *
     * @return An ongoing builder.
     */
    AsyncAdvancedProfileControllerBuilder &notParentedToCurrentTask();

    /**
     * Builds the AsyncAdvancedProfileController.
     *
     * @return A fully built AsyncAdvancedProfileController.
     */
    std::shared_ptr<AsyncAdvancedProfileController> buildAdvancedProfileController();

private:
    std::shared_ptr<Logger> logger;

    bool hasConfig{false};
    planner::PlannerConfig config;

    bool hasChassisController{false};
    std::shared_ptr<ChassisController> chassisController;
    TimeUtilFactory timeUtilFactory = TimeUtilFactory();
    std::shared_ptr<Logger> controllerLogger = Logger::getDefaultLogger();

    bool isParentedToCurrentTask{true};
};
