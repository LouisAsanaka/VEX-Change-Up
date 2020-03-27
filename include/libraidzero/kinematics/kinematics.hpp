#pragma once

#include "main.h"
#include "libraidzero/kinematics/chassisSpeeds.hpp"
#include "libraidzero/kinematics/driveWheelSpeeds.hpp"

class Kinematics {
public:
    explicit Kinematics(ChassisScales iscales) : wheelTrack{iscales.wheelTrack} {}

    /**
     * Returns a chassis speed from left and right component velocities using
     * forward kinematics.
     *
     * @param wheelSpeeds The left and right velocities.
     * @return The chassis speed.
     */
    constexpr ChassisSpeeds toChassisSpeeds(const DriveWheelSpeeds& wheelSpeeds) const {
        return {(wheelSpeeds.left + wheelSpeeds.right) / 2.0, 0_mps,
                (wheelSpeeds.right - wheelSpeeds.left) / wheelTrack * 1_rad};
    }

    /**
     * Returns left and right component velocities from a chassis speed using
     * inverse kinematics.
     * NOTE: +omega is counter-clockwise
     *
     * @param chassisSpeeds The linear and angular (dx and dtheta) components that
     * represent the chassis' speed.
     * @return The left and right velocities.
     */
    constexpr DriveWheelSpeeds toWheelSpeeds(const ChassisSpeeds& chassisSpeeds) const {
        return {chassisSpeeds.vx - wheelTrack / 2 * chassisSpeeds.omega / 1_rad,
                chassisSpeeds.vx + wheelTrack / 2 * chassisSpeeds.omega / 1_rad};
    }

    QLength wheelTrack;
};