#include "libraidzero/geometry/transform2d.hpp"
#include "libraidzero/geometry/pose2d.hpp"

Transform2d::Transform2d(Pose2d initial, Pose2d final) {
    // We are rotating the difference between the translations
    // using a clockwise rotation matrix. This transforms the global
    // delta into a local delta (relative to the initial pose).
    m_translation = (final.translation() - initial.translation())
                        .rotateBy(-initial.rotation());

    m_rotation = final.rotation() - initial.rotation();
}

Transform2d::Transform2d(Translation2d translation, Rotation2d rotation)
    : m_translation(translation), m_rotation(rotation) {}

bool Transform2d::operator==(const Transform2d& other) const {
    return m_translation == other.m_translation && m_rotation == other.m_rotation;
}

bool Transform2d::operator!=(const Transform2d& other) const {
    return !operator==(other);
}
