#include "controller/ramseteController.hpp"
#include "main.h"
#include "kinematics/chassisSpeeds.hpp"
#include <numeric>

/**
 * Returns sin(x) / x.
 *
 * @param x Value of which to take sinc(x).
 */
static double sinc(double x) {
    if (std::abs(x) < 1e-9) {
        return 1.0 - 1.0 / 6.0 * x * x;
    } else {
        return std::sin(x) / x;
    }
}

RamseteController::RamseteController(double ib, double izeta)
    : b(ib), zeta(izeta) {}

ChassisSpeeds RamseteController::calculate(const Pose2d& currentPose, const Pose2d& poseRef,
    QSpeed linearVelocityRef, QAngularSpeed angularVelocityRef)
{
    if (!enabled) {
        return ChassisSpeeds{linearVelocityRef, 0_mps, angularVelocityRef};
    }

    poseError = poseRef.relativeTo(currentPose);

    // Aliases for equation readability
    double eX = poseError.translation().x().convert(meter);
    double eY = poseError.translation().y().convert(meter);
    double eTheta = poseError.rotation().angle().convert(radian);
    double vRef = linearVelocityRef.convert(mps);
    double omegaRef = angularVelocityRef.convert(radps);

    double k =
        2.0 * zeta * std::sqrt(std::pow(omegaRef, 2) + b * std::pow(vRef, 2));

    double v{vRef * poseError.rotation().cos() + k * eX};
    double omega{omegaRef + k * eTheta + b * vRef * sinc(eTheta) * eY};
    return ChassisSpeeds{v * mps, 0_mps, omega * radps};
}

void RamseteController::setGains(double ib, double izeta) {
    if (ib) {
        b = ib;
    }
    if (izeta) {
        zeta = izeta;
    }
}

void RamseteController::setEnabled(bool status) {
    enabled = status;
}
