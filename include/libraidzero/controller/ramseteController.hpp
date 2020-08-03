#pragma once

#include "libraidzero/kinematics/chassisSpeeds.hpp"
#include "libraidzero/geometry/pose2d.hpp"
#include "okapi/api/units/QAngularSpeed.hpp"
#include "okapi/api/units/QSpeed.hpp"

class RamseteController {
public:
	/**
	 * b and ζ are tuning parameters where b > 0 and ζ ∈ (0, 1). Larger values of
	 * b make convergence more aggressive (like a proportional term), and larger
	 * values of ζ provide more damping
	 */
	RamseteController(double ib, double izeta);

    /**
     * Default arguments have been well-tested
     */
    RamseteController() : RamseteController{2.0, 0.7} {};

	/**
     * Returns the next output of the Ramsete controller.
     *
     * The reference pose, linear velocity, and angular velocity should come from
     * a drivetrain trajectory.
     *
     * @param currentPose        The current pose.
     * @param poseRef            The desired pose.
     * @param linearVelocityRef  The desired linear velocity.
     * @param angularVelocityRef The desired angular velocity.
     */
    ChassisSpeeds calculate(const Pose2d& currentPose, const Pose2d& poseRef,
        okapi::QSpeed linearVelocityRef,
        okapi::QAngularSpeed angularVelocityRef);

	void setGains(double ib, double izeta);

    void setEnabled(bool status);

private:
    double b, zeta;

	Pose2d poseError;
    Pose2d poseTolerance;

    bool enabled = true;
};
