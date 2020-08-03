#pragma once

#include "libraidzero/controller/asyncRamsetePathController.hpp"
#include "libraidzero/controller/util/ramseteUtil.hpp"

class AsyncRamsetePathControllerBuilder {
public:
    /**
     * A builder that creates async Ramsete path controllers. Use this to build an
     * AsyncRamsetePathController.
     *
     * @param ilogger The logger this instance will log to.
     */
    explicit AsyncRamsetePathControllerBuilder(
        std::shared_ptr<Logger> ilogger = Logger::getDefaultLogger());

    /**
     * Sets the output. This must be used with buildRamsetePathController().
     *
     * @param icontroller The chassis controller to use.
     * @return An ongoing builder.
     */
    AsyncRamsetePathControllerBuilder &
    withOutput(const std::shared_ptr<OdomChassisController> &icontroller);

    /**
     * Sets the limits.
     *
     * @param ilimits The limits.
     * @return An ongoing builder.
     */
    AsyncRamsetePathControllerBuilder &withLimits(const PathfinderLimits &ilimits);

    /**
     * Sets the Ramsete controller constants.
     *
     * @param iconstants The constants.
     * @return An ongoing builder.
     */
    AsyncRamsetePathControllerBuilder &withConstants(const RamseteConstants &iconstants);

    /**
     * Sets the TimeUtilFactory used when building the controller. The default is the static
     * TimeUtilFactory.
     *
     * @param itimeUtilFactory The TimeUtilFactory.
     * @return An ongoing builder.
     */
    AsyncRamsetePathControllerBuilder &withTimeUtilFactory(const TimeUtilFactory &itimeUtilFactory);

    /**
     * Sets the logger.
     *
     * @param ilogger The logger.
     * @return An ongoing builder.
     */
    AsyncRamsetePathControllerBuilder &withLogger(const std::shared_ptr<Logger> &ilogger);

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
    AsyncRamsetePathControllerBuilder &parentedToCurrentTask();

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
    AsyncRamsetePathControllerBuilder &notParentedToCurrentTask();

    /**
     * Builds the AsyncRamsetePathController.
     *
     * @return A fully built AsyncRamsetePathController.
     */
    std::shared_ptr<AsyncRamsetePathController> buildRamsetePathController();

private:
    std::shared_ptr<Logger> logger;

    bool hasLimits{false};
    PathfinderLimits limits;

    RamseteConstants constants{};

    bool hasChassisController{false};
    std::shared_ptr<OdomChassisController> chassisController;
    TimeUtilFactory timeUtilFactory = TimeUtilFactory();
    std::shared_ptr<Logger> controllerLogger = Logger::getDefaultLogger();

    bool isParentedToCurrentTask{true};
};
