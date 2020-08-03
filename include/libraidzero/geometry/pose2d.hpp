#pragma once

#include "libraidzero/geometry/transform2d.hpp"
#include "libraidzero/geometry/translation2d.hpp"
#include "okapi/api/odometry/odomState.hpp"
#include "okapi/api/units/QLength.hpp"
#include <sstream>

/**
 * Represents a 2d pose containing translational and rotational elements.
 */
class Pose2d {
public:
    /**
     * Constructs a pose at the origin facing toward the positive X axis.
     * (Translation2d{0, 0} and Rotation{0})
     */
    constexpr Pose2d() = default;

    /**
     * Constructs a pose with the specified translation and rotation.
     *
     * @param translation The translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    Pose2d(Translation2d translation, Rotation2d rotation);

    /**
     * Convenience constructors that takes in x and y values directly instead of
     * having to construct a Translation2d.
     *
     * @param x The x component of the translational component of the pose.
     * @param y The y component of the translational component of the pose.
     * @param rotation The rotational component of the pose.
     */
    Pose2d(okapi::QLength x, okapi::QLength y, Rotation2d rotation);

    /**
     * Transforms the pose by the given transformation and returns the new
     * transformed pose.
     *
     * [x_new]    [cos, -sin, 0][transform.x]
     * [y_new] += [sin,  cos, 0][transform.y]
     * [t_new]    [0,    0,   1][transform.t]
     *
     * @param other The transform to transform the pose by.
     *
     * @return The transformed pose.
     */
    Pose2d operator+(const Transform2d& other) const;

    /**
     * Transforms the current pose by the transformation.
     *
     * This is similar to the + operator, except that it mutates the current
     * object.
     *
     * @param other The transform to transform the pose by.
     *
     * @return Reference to the new mutated object.
     */
    Pose2d& operator+=(const Transform2d& other);

    /**
     * Returns the Transform2d that maps the one pose to another.
     *
     * @param other The initial pose of the transformation.
     * @return The transform that maps the other pose to the current pose.
     */
    Transform2d operator-(const Pose2d& other) const;

    /**
     * Checks equality between this Pose2d and another object.
     *
     * @param other The other object.
     * @return Whether the two objects are equal.
     */
    bool operator==(const Pose2d& other) const;

    /**
     * Checks inequality between this Pose2d and another object.
     *
     * @param other The other object.
     * @return Whether the two objects are not equal.
     */
    bool operator!=(const Pose2d& other) const;

    /**
     * Returns the underlying translation.
     *
     * @return Reference to the translational component of the pose.
     */
    const Translation2d& translation() const { return m_translation; }

    /**
     * Returns the underlying rotation.
     *
     * @return Reference to the rotational component of the pose.
     */
    const Rotation2d& rotation() const { return m_rotation; }

    /**
     * Transforms the pose by the given transformation and returns the new pose.
     * See + operator for the matrix multiplication performed.
     *
     * @param other The transform to transform the pose by.
     *
     * @return The transformed pose.
     */
    Pose2d transformBy(const Transform2d& other) const;

    /**
     * Returns the other pose relative to the current pose.
     *
     * This function can often be used for trajectory tracking or pose
     * stabilization algorithms to get the error between the reference and the
     * current pose.
     *
     * @param other The pose that is the origin of the new coordinate frame that
     * the current pose will be converted into.
     *
     * @return The current pose relative to the new origin pose.
     */
    Pose2d relativeTo(const Pose2d& other) const;

    static Pose2d fromOdomState(const okapi::OdomState& state);

    std::string toString() const {
        std::stringstream ss;
        ss << "x: " << m_translation.x().convert(okapi::meter) << "m, ";
        ss << "y: " << m_translation.y().convert(okapi::meter) << "m, ";
        ss << "theta: " << m_rotation.angle().convert(okapi::degree) << "deg";
        return ss.str();
    }
private:
    Translation2d m_translation;
    Rotation2d m_rotation;
};
