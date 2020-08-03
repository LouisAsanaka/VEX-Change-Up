#include "libraidzero/controller/ramseteController.hpp"
#include "libraidzero/kinematics/chassisSpeeds.hpp"
#include <numeric>

/**
 * Returns sin(x) / x.
 *
 * @param x Value of which to take sinc(x).
 */
static double sinc(double x) {
    if (std::abs(x) < 1e-9) {
        return 1.0 - 1.0 / 6.0 * x * x;
    }
    return std::sin(x) / x;
}

RamseteController::RamseteController(double ib, double izeta)
    : b(ib), zeta(izeta) {}

ChassisSpeeds RamseteController::calculate(const Pose2d& currentPose, const Pose2d& poseRef,
    okapi::QSpeed linearVelocityRef, okapi::QAngularSpeed angularVelocityRef)
{
    if (!enabled) {
        return ChassisSpeeds{linearVelocityRef, 0_mps, angularVelocityRef};
    }

    poseError = poseRef.relativeTo(currentPose);

    // Aliases for equation readability
    double eX = poseError.translation().x().convert(okapi::meter);
    double eY = poseError.translation().y().convert(okapi::meter);
    double eTheta = poseError.rotation().angle().convert(okapi::radian);
    double vRef = linearVelocityRef.convert(okapi::mps);
    double omegaRef = angularVelocityRef.convert(okapi::radps);

    double k =
        2.0 * zeta * std::sqrt(std::pow(omegaRef, 2) + b * std::pow(vRef, 2));

    double v{vRef * poseError.rotation().cos() + k * eX};
    double omega{omegaRef + k * eTheta + b * vRef * sinc(eTheta) * eY};
    return ChassisSpeeds{v * okapi::mps, 0_mps, omega * okapi::radps};
}

void RamseteController::setGains(double ib, double izeta) {
    if (ib != 0.0) {
        b = ib;
    }
    if (izeta != 0.0) {
        zeta = izeta;
    }
}

void RamseteController::setEnabled(bool status) {
    enabled = status;
}
