#include "libraidzero/geometry/pose2d.hpp"

#include <cmath>

Pose2d::Pose2d(Translation2d translation, Rotation2d rotation)
    : m_translation(translation), m_rotation(rotation) {}

Pose2d::Pose2d(okapi::QLength x, okapi::QLength y, Rotation2d rotation)
    : m_translation(x, y), m_rotation(rotation) {}

Pose2d Pose2d::operator+(const Transform2d& other) const {
    return transformBy(other);
}

Pose2d& Pose2d::operator+=(const Transform2d& other) {
    m_translation += other.translation().rotateBy(m_rotation);
    m_rotation += other.rotation();
    return *this;
}

Transform2d Pose2d::operator-(const Pose2d& other) const {
    const auto pose = this->relativeTo(other);
    return Transform2d(pose.translation(), pose.rotation());
}

bool Pose2d::operator==(const Pose2d& other) const {
    return m_translation == other.m_translation && m_rotation == other.m_rotation;
}

bool Pose2d::operator!=(const Pose2d& other) const {
    return !operator==(other);
}

Pose2d Pose2d::transformBy(const Transform2d& other) const {
    return {m_translation + (other.translation().rotateBy(m_rotation)),
            m_rotation + other.rotation()};
}

Pose2d Pose2d::relativeTo(const Pose2d& other) const {
    const Transform2d transform{other, *this};
    return {transform.translation(), transform.rotation()};
}

Pose2d Pose2d::fromOdomState(const okapi::OdomState& state) {
    return {Translation2d{state.x, state.y}, Rotation2d{-1 * state.theta}};
}